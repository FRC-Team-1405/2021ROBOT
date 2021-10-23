/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.commands.DriveByAngle;
import frc.robot.commands.ShootContinous;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.ZeroizeOdometry;
import frc.robot.commands.ZeroizeSwerveModules;
import frc.robot.lib.CustomPIDController;
import frc.robot.lib.DistanceToAngle;
import frc.robot.lib.DistanceToPower;
import frc.robot.lib.MathTools;
import frc.robot.lib.SmartBooleanSupplier;
import frc.robot.lib.SmartSupplier;
import frc.robot.lib.thirdcoast.swerve.SwerveDrive.DriveMode;
import frc.robot.sensors.LidarLitePWM;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDriveBase;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here... 

  private static final Logger logger = Logger.getLogger(RobotContainer.class.getName());

  // private final ArcadeDrive driveBase = new ArcadeDrive();
  public final SwerveDriveBase swerveDriveBase = new SwerveDriveBase(); 
  private Limelight limelight = new Limelight(new Limelight.Position(41.91, 30.12));

  private XboxController driver = new XboxController(Constants.pilot); 
  private XboxController operator = new XboxController(Constants.operator); 
  
  private Intake intake = new Intake(); 
  private Shooter shooter = new Shooter(); 
  private Hood hood = new Hood(); 
  private Climber climber = new Climber();
  private LidarLitePWM aimingLidar = new LidarLitePWM(new DigitalInput(8));

  private double speedLimit = new SmartSupplier("Drivebase/SpeedLimit", 0.35).getAsDouble();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureDriverButtonBindings(); 
    configureOperatorButtonBindings(); 
    configureDemoButtonBindings();
    initShuffleBoard();
    limelight.setPipeline((byte) 0);
    limelight.setLED(Limelight.LED.Off); 

    swerveDriveBase.stop();
    swerveDriveBase.zeroAzimuthEncoders();
    swerveDriveBase.zeroGyro();

    var hoodControl = new RunCommand( () -> { hood.setPosition((int) DistanceToAngle.calculate(distanceToTarget())); }, hood );
    hoodControl.withName("Dynamic Hood Control");
    hood.setDefaultCommand( hoodControl ); 
    
    var climberControl = new RunCommand( () -> {
      double left = operator.getY(Hand.kLeft); 
      double right = operator.getY(Hand.kRight); 
      climber.moveLeft(left < Math.abs(.35) ? 0 : left); 
      climber.moveRight(right < Math.abs(.35) ? 0 : right);
    }, climber); 
    climberControl.setName("Climber Control"); 
    climber.setDefaultCommand(climberControl);

/*
    switch(joystickSelector.getSelected()){
      case "Logitech":
        swerveDriveBase.setDefaultCommand( new SwerveDrive( this::getForwardSwerve, 
                                                            this::getStrafeSwerve, 
                                                            this::getYawSwerveLogitech, 
                                                            this::getSpeedLimitLogitech,
                                                            swerveDriveBase) ); 
                                                            break;
      case "XboxController": 
      default:
        swerveDriveBase.setDefaultCommand( new SwerveDrive( this::getForwardSwerve, 
                                                            this::getStrafeSwerve, 
                                                            this::getYawSwerveXboxController, 
                                                            this::getSpeedLimitXboxController,
                                                            swerveDriveBase) );
    }
*/
    
    SmartDashboard.putBoolean("Lidar Ready", aimingLidar.getDistance() > 0); 

  } 

  public boolean shooterOverrideEnabled(){ 
    return operator.getBumper(Hand.kRight); 
  }

  public double getForwardSwerve() {
    return -driver.getY(Hand.kLeft); 
  }

  /** Left stick Y (left-right) axis. */
  public double getStrafeSwerve() {
    return -driver.getX(Hand.kLeft);
  } 

  /** Right stick Y (left-right) axis. */
  public double getYawSwerveXboxController() {
    return -driver.getX(Hand.kRight); 
  }  

  public double getYawSwerveLogitech() { 
    double yaw = driver.getRawAxis(2);
    if (Math.abs(yaw) < Constants.deadBand){
      return 0.0;
    } 
    return yaw;
  }

  public double getSpeedLimitXboxController() { 
    return MathTools.map(driver.getTriggerAxis(Hand.kRight), 0.0, 1.0, speedLimit, 1.0);
  } 

  public double getSpeedLimitLogitech() { 
    return MathTools.map(driver.getRawAxis(3), -1, 1, 1.0, speedLimit); 
  }

  SendableChooser<Integer> autoSelector; 
   
  private void initShuffleBoard(){
    boolean isLogitech = new SmartBooleanSupplier("Use Logitech Controller", false).getAsBoolean(); 

    autoSelector = new SendableChooser<Integer>();
    autoSelector.addOption("Do nothing", 0);
    autoSelector.addOption("Drive forward.", 1);
    autoSelector.addOption("Shoot then drive", 2);
    autoSelector.addOption("Swerve Test Auto", 3); 
    autoSelector.addOption("Slalom", 4); 
    autoSelector.addOption("Waypoint Slalom", 5); 
    autoSelector.addOption("Circle", 6); 
    autoSelector.addOption("Search Path A Red", 7); 
    autoSelector.addOption("Search Path B Red", 8); 
    autoSelector.addOption("Bounce", 9);
    autoSelector.setDefaultOption("Do nothing", 0); 
    
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto") ;
    autoTab.add(autoSelector)
            .withWidget(BuiltInWidgets.kComboBoxChooser);
    SmartDashboard.putNumber("Auto/Selected_Auto", 1);
    autoTab.add("Auto/Initial_Delay", 0); 

    InstantCommand setPosition;
    setPosition = new InstantCommand(() -> {
                          double distanceMeters = aimingLidar.getDistance()/100.0;
                          swerveDriveBase.resetOdometry( new Pose2d(-distanceMeters,  3.1, swerveDriveBase.getPose().getRotation()) );
                        }) ;
    setPosition.setName("Left");
    autoTab.add( setPosition );

    setPosition = new InstantCommand(() -> {
                          double distanceMeters = aimingLidar.getDistance()/100.0;
                          swerveDriveBase.resetOdometry( new Pose2d(-distanceMeters,  0.0, swerveDriveBase.getPose().getRotation()) );
                        }) ;
    setPosition.setName("Center");
    autoTab.add( setPosition );

    setPosition = new InstantCommand(() -> {
                          double distanceMeters = aimingLidar.getDistance()/100.0;
                          swerveDriveBase.resetOdometry( new Pose2d(-distanceMeters, -1.6, swerveDriveBase.getPose().getRotation()) );
                        }) ;
    setPosition.setName("Right");
    autoTab.add( setPosition );
  
    if(!Robot.fmsAttached){
      ShuffleboardTab testCommandsTab = Shuffleboard.getTab("Test Commands"); 
      testCommandsTab.add(new ZeroizeSwerveModules(swerveDriveBase)); 
      testCommandsTab.add(new ZeroizeOdometry(swerveDriveBase));

      // Test Fire Command
      SmartDashboard.putNumber("Shooter/Power", 0);
      SmartDashboard.putNumber("Shooter/Angle", 0);
      var testFire = new ShootContinous(  shooter, 
                                          hood, 
                                          () -> { return SmartDashboard.getNumber("Shooter/Power",0);}, 
                                          () -> { return SmartDashboard.getNumber("Shooter/Angle",0);},
                                          limelight,
                                          true, 
                                          this::shooterOverrideEnabled
                                          );
      testFire.withName("Test Fire"); 
      testCommandsTab.add( testFire );

      // Measure distance & angle
      SmartDashboard.putNumber("Shooter/Distance", 0);
      SmartDashboard.putNumber("Shooter/Angle", 0);
      var readDistanceAngle = new RunCommand( () -> { 
        SmartDashboard.putNumber("Shooter/Distance", distanceToTarget()); 
        SmartDashboard.putNumber("Shooter/Angle", angleToTarget());
      });
      readDistanceAngle.withName("Distance & Angle");
      testCommandsTab.add( readDistanceAngle );

      // Enable Target Camera
      var targetCamera = new StartEndCommand( 
          () -> { 
            limelight.setPipeline(Constants.LimelightConfig.TargetPipeline);
            limelight.setLED(Limelight.LED.Default);
          },
          () -> {
            limelight.setLED(Limelight.LED.Off);
          });
      targetCamera.withName("Camera Target");
      testCommandsTab.add( targetCamera );
      
      // Test turn to angle by turning to zero
      double robotAngle = swerveDriveBase.getPose().getRotation().getDegrees();
      DoubleSupplier returnZero = () -> { return 0.0; };
      var turnToAngle = new DriveByAngle( returnZero, 
                              returnZero, 
                              isLogitech ? this::getSpeedLimitLogitech : this::getSpeedLimitXboxController,
                              angleToFieldAngle(0.0), 
                              swerveDriveBase);
      turnToAngle.withName("Test Turn");
      testCommandsTab.add( turnToAngle );

      // RunCommand readDistance = new RunCommand(lidar::readDistance);
      // readDistance.setName("Read_Distance");
      // testCommandsTab.add(readDistance);

      // SmartDashboard.putNumber("Turret/turnAngle", 0);
      // InstantCommand turnTurret = new InstantCommand( () -> {launcher.turnTurret(SmartDashboard.getNumber("Turret/turnAngle", 0)); });
      // turnTurret.setName("Turn_Turret");
      // testCommandsTab.add(turnTurret);
  
      // RunCommand readColor = new RunCommand(colorSensor::readColor);
      // readColor.setName("Read_Color");
      // testCommandsTab.add(readColor);
  
      // InstantCommand resetPosition = new InstantCommand( () -> { driveBase.resetPosition(); } );
      // resetPosition.setName("Reset Position");
      // testCommandsTab.add(resetPosition);
  
      // InstantCommand stopFlywheels = new InstantCommand( launcher::stopFlywheels );
      // stopFlywheels.setName("Stop Flywheels");
      // testCommandsTab.add(stopFlywheels);
  
      // SmartDashboard.putNumber("Shooter/Elevation", 0);
      // InstantCommand setElevation = new InstantCommand( () -> {launcher.setElevationManual(SmartDashboard.getNumber("Shooter/Elevation", 0)); });
      // setElevation.setName("Set Elevation");
      // testCommandsTab.add(setElevation);

      // InstantCommand resetTurret = new InstantCommand(launcher::resetEncoder);
      // resetTurret.setName("Reset Turret");
      // testCommandsTab.add(resetTurret);

      InstantCommand resetScizzorsCommand = new InstantCommand(climber::resetClimberEncoders);
      resetScizzorsCommand.setName("Reset Scizzors");
      testCommandsTab.add(resetScizzorsCommand);

      // InstantCommand toggleClimbLimits = new InstantCommand(climber::toggleLimits);
      // toggleClimbLimits.setName("Toggle Climb Limits");
      // testCommandsTab.add(toggleClimbLimits);
    }

    SmartDashboard.putData( new PowerDistributionPanel(Constants.PDP) );
  }

  
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureDriverButtonBindings() {
    boolean isLogitech = new SmartBooleanSupplier("Use Logitech Controller", false).getAsBoolean(); 
    /**
     * Driver:
     * +Left joystick: swerve forward/reverse
     * +Right joystick: swerve left/right
     * +Right joystick press: face target
     * +A: Climber to Home position
     * +B: turn 180
     * +X: Climber to Climb position
     * +Y: Climber to Grab position
     * +Right bumper: intake
     * +Left bumper: intake deploy/retract
     * +Right trigger: increase shooter distance
     * +Left trigger: decrease shooter distance
     * +D-pad up: 
     * +D-pad down: 
     * +D-pad left:
     * +D-pad right:
     * +Start & Back: lock | unlock climb controls 
     */
    swerveDriveBase.setDefaultCommand( new SwerveDrive( this::getForwardSwerve, 
                                                        this::getStrafeSwerve, 
                                                        isLogitech ? this::getYawSwerveLogitech : this::getYawSwerveXboxController, 
                                                        isLogitech ? this::getSpeedLimitLogitech : this::getSpeedLimitXboxController,
                                                        swerveDriveBase) ); 

    new JoystickButton(driver, XboxController.Button.kBumperRight.value) 
          .whenPressed(new InstantCommand(intake::enable, intake)) 
          .whenReleased(new InstantCommand(intake::disable, intake)); 
    
    SmartDashboard.putBoolean("Intake Deployed", intake.isDeployed());
    new JoystickButton(driver, XboxController.Button.kBumperLeft.value) 
          .whenPressed( new InstantCommand( () -> {
            if (intake.isDeployed()){
              intake.retract();
            } else {
              intake.deploy();
            }
            SmartDashboard.putBoolean("Intake Deployed", intake.isDeployed());
          }));

    new JoystickButton(driver, XboxController.Button.kStickRight.value) 
          .whileHeld( new DriveByAngle( this::getForwardSwerve, 
                                        this::getStrafeSwerve, 
                                        isLogitech ? this::getSpeedLimitLogitech : this::getSpeedLimitXboxController,
                                        this::angleToTarget, 
                                        swerveDriveBase) );
                        
    new JoystickButton(driver, XboxController.Button.kB.value)
          .whileHeld( new DriveByAngle( this::getForwardSwerve,
                                        this::getStrafeSwerve,
                                        isLogitech ? this::getStrafeSwerve : this::getSpeedLimitXboxController,
                                        this.angleToDeltaAngle(180.0),
                                        swerveDriveBase) );

    // lock / unlock the climb controls
    new JoystickButton(driver, XboxController.Button.kBack.value)
      .and( new JoystickButton(driver, XboxController.Button.kStart.value) )
      .whenActive( new InstantCommand( () -> {
        intake.LockDeployed();
        climber.toggleEnable();
      }));

    new JoystickButton(driver, XboxController.Button.kY.value)
      .whenHeld( new FunctionalCommand( () -> { climber.reachUp(); }, 
                                           () -> { climber.reachUp(); }, 
                                           (interrupted) -> { climber.stop(); }, 
                                           () -> { return false; }, 
                                           climber));

    new JoystickButton(driver, XboxController.Button.kA.value)
      .whenHeld( new FunctionalCommand( () -> { climber.goHome(); }, 
                                           () -> { climber.goHome(); }, 
                                           (interrupted) -> { climber.stop(); }, 
                                           () -> { return false; }, 
                                           climber));

    new JoystickButton(driver, XboxController.Button.kX.value)
      .whenHeld( new FunctionalCommand( () -> { climber.reachLow(); }, 
                                           () -> { climber.reachLow(); }, 
                                           (interrupted) -> { climber.stop(); }, 
                                           () -> { return false; }, 
                                           climber));
    }; 
  
  private void configureOperatorButtonBindings(){ 
    /**
     * Operator:
     * +Right bumper: shoot now
     * 
     * +A: sample distance once then shoot
     * +B: shoot from the trench
     * +X: shoot to the trench
     * +Y: sample distance continuos and shoot
     * +Left trigger: 
     * +Right trigger: 
     * +D-pad up: 
     * +D-pad down: 
     * +D-pad left: 
     * +D-pad right: 
     * +Left joystick:
     * +Right joystick:
     * +Start & Back: lock | unlock climb controls 
     */
    new JoystickButton(driver, XboxController.Button.kBumperLeft.value)
      .whenPressed(new SequentialCommandGroup(new InstantCommand(shooter::index), new WaitCommand(.5), new InstantCommand(shooter::close)));    
      
    new JoystickButton(operator, XboxController.Button.kA.value)
      .whileHeld( new ShootContinous( shooter, 
                                      hood, 
                                      () -> { return DistanceToPower.calculate(distanceToTarget());}, 
                                      () -> { return DistanceToAngle.calculate(distanceToTarget());},
                                      limelight,
                                      false, 
                                      this::shooterOverrideEnabled
                                      )); 
     
    new JoystickButton(operator, XboxController.Button.kY.value)
      .whileHeld( new ShootContinous( shooter, 
                                      hood, 
                                      () -> { return DistanceToPower.calculate(distanceToTarget());}, 
                                      () -> { return DistanceToAngle.calculate(distanceToTarget());},
                                      limelight,
                                      true, 
                                      this::shooterOverrideEnabled
                                      )); 

    // shoot into the trench
    // TODO verify power and angle for a low into trench shot
    // TODO or a high shot into the trench
    new JoystickButton(operator, XboxController.Button.kX.value)
      .whileHeld( new ShootContinous(  shooter, 
                                      hood, 
                                      () -> { return DistanceToPower.calculate(Constants.Hood.LoadingToTrench)+distanceOffset();}, 
                                      () -> { return Constants.Hood.maxAngle;},
                                      limelight,
                                      true, 
                                      this::shooterOverrideEnabled
                                      )); 
    // shoot from the trench
    new JoystickButton(operator, XboxController.Button.kB.value)
      .whileHeld( new ShootContinous(  shooter, 
                                      hood, 
                                      () -> { return DistanceToPower.calculate(Constants.Hood.TrenchToTarget+distanceOffset());}, 
                                      () -> { return DistanceToAngle.calculate(Constants.Hood.TrenchToTarget+distanceOffset());},
                                      limelight,
                                      true, 
                                      this::shooterOverrideEnabled
                                      )); 

    // lock / unlock the climb controls
    new JoystickButton(operator, XboxController.Button.kBack.value)
      .and( new JoystickButton(operator, XboxController.Button.kStart.value) )
      .whenActive( new InstantCommand( () -> {
        climber.toggleEnable();
      }));
  }

  private void configureDemoButtonBindings(){ 
    XboxController demo = new XboxController(Constants.demoController);
    new JoystickButton(demo, XboxController.Button.kX.value)
      .whenPressed( new InstantCommand( () -> {
        intake.deploy();
      }));
    new JoystickButton(demo, XboxController.Button.kB.value)
      .whenPressed( new InstantCommand( () -> {
        intake.retract();
      }));
  }

  private DoubleSupplier angleToDeltaAngle(double deltaAngle){
    double targetAngle = swerveDriveBase.getPose().getRotation().getDegrees()+deltaAngle; 
    return () -> { 
      double robotAngle = swerveDriveBase.getPose().getRotation().getDegrees(); 
      return Math.IEEEremainder(robotAngle-targetAngle, 360.0);
    };
  }

  private DoubleSupplier angleToFieldAngle(double fieldAngle){
    return () -> { 
      double robotAngle = swerveDriveBase.getPose().getRotation().getDegrees(); 
      return Math.IEEEremainder(robotAngle-fieldAngle, 360.0);
    };
  }

  private double angleToTarget(){
    Pose2d robotPosition = swerveDriveBase.getPose() ;
    double robotAngle = robotPosition.getRotation().getDegrees(); 

    String odometryAngle  = "N/A";
    String limelightAngle = "N/A";
    double value;
    
    value = robotPosition.getX() == 0.0
                ? 0.0
                : robotAngle - Math.toDegrees( Math.atan( robotPosition.getY() / robotPosition.getX() )) ;
    odometryAngle = String.format("%.1f", value);

    if (limelight.getPipeline() == Constants.LimelightConfig.TargetPipeline && limelight.hasTarget()) {
      value = limelight.getTA();
      odometryAngle = String.format("%.1f", value);
    }

    SmartDashboard.putString("Angle/Odomentry",  odometryAngle) ;
    SmartDashboard.putString("Angle/Limelight",  limelightAngle) ;
    return value;
  }

  private double distanceOffset(){
    return -operator.getTriggerAxis(Hand.kLeft)*100.0 + operator.getTriggerAxis(Hand.kRight)*100;
  }

  private double distanceToTarget(){
    double odometryDistance = Math.sqrt( Math.pow(swerveDriveBase.getPose().getX(), 2) + Math.pow(swerveDriveBase.getPose().getY(), 2) ) * 100.0;
    double lidarDistance = aimingLidar.getDistance();
    double limelightDistance = limelight.getPipeline() == Constants.LimelightConfig.TargetPipeline ? limelight.fixedAngleDist(243.84) : Integer.MIN_VALUE;

    double offset = distanceOffset();
    SmartDashboard.putNumber("Distance/Offset",     offset) ;
    SmartDashboard.putString("Distance/Odomentry",  odometryDistance   >= 0 ? String.format("%.0f",odometryDistance)  : "N/A") ;
    SmartDashboard.putString("Distance/Lidar",      lidarDistance      >= 0 ? String.format("%.0f",lidarDistance)     : "N/A") ;
    SmartDashboard.putString("Distance/Limelight",  limelightDistance  >= 0 ? String.format("%.0f",limelightDistance) : "N/A") ;

    if ( Math.abs(odometryDistance - lidarDistance) > 100.0){
      lidarDistance = Integer.MIN_VALUE; // assume an object is blocking the lidar
    }

    if (lidarDistance >= 0)      return lidarDistance     + offset;
    if (limelightDistance >= 0)  return limelightDistance + offset;
    if (odometryDistance >= 0)   return odometryDistance  + offset;

    return Integer.MIN_VALUE; 
  }

  // An example selector method for the selectcommand.  Returns the selector that will select
  // which command to run.  Can base this choice on logical conditions evaluated at runtime.
  
  private int select() { 
    return (int) autoSelector.getSelected();
  }

  // An example selectcommand.  Will select from the three commands based on the value returned
  // by the selector method at runtime.  Note that selectcommand works on Object(), so the
  // selector does not have to be an enum; it could be any desired type (string, integer,
  // boolean, double...)
  private final Command selectCommand =
      new SelectCommand(
          // Maps selector values to commands
          Map.ofEntries(
              Map.entry(0, new PrintCommand("Do nothing")),
              //Map.entry(1, auto1),
              Map.entry(2, shootThenDrive()),
              Map.entry(3, getSwerveAutoTest()), 
              Map.entry(4, slalom()), 
              Map.entry(5, waypointSlalom()), 
              Map.entry(6, Barrel()), 
              Map.entry(7, galacticSearchPathARed()), 
              Map.entry(8, galacticSearchPathBRed()), 
              Map.entry(9, bounce())
          ),
          this::select
      );

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoCommand;
    return selectCommand;
  } 

  private Command runTrajecotory(Trajectory trajectory){ 
    if (swerveDriveBase == null){
      return new PrintCommand("Wrong drivebase -- expecting swerve");
    }  

    //trajectory = trajectory.relativeTo(swerveDriveBase.getPose());

    var thetaController =
    new ProfiledPIDController(
        new SmartSupplier("Swerve rP", 0).getAsDouble(),
        new SmartSupplier("Swerve rI", 0).getAsDouble(), 
        new SmartSupplier("Swerve rD", 0).getAsDouble(), 
        new TrapezoidProfile.Constraints(Constants.SwerveBase.maxAngularSpeed, Constants.SwerveBase.maxAngularAccelerartion));
thetaController.enableContinuousInput(-Math.PI, Math.PI);

SwerveControllerCommand swerveControllerCommand =
    new SwerveControllerCommand(
        trajectory,
        swerveDriveBase::getPose, // Functional interface to feed supplier
        swerveDriveBase.getKinematics(),

        // Position controllers
        new CustomPIDController( "X",
                            new SmartSupplier("Swerve xP", 0).getAsDouble(),
                            new SmartSupplier("Swerve xI", 0).getAsDouble(), 
                            new SmartSupplier("Swerve xD", 0).getAsDouble()),  
        new CustomPIDController( "Y",  
                            new SmartSupplier("Swerve yP", 0).getAsDouble(),
                            new SmartSupplier("Swerve yI", 0).getAsDouble(), 
                            new SmartSupplier("Swerve yD", 0).getAsDouble()), 
                            
        thetaController,
        swerveDriveBase::setModuleStates,
        swerveDriveBase); 


// Run path following command, then stop at the end.
return new SequentialCommandGroup( 
  new InstantCommand(() -> {
    // Reset odometry to the starting pose of the trajectory.
    swerveDriveBase.resetOdometry(trajectory.getInitialPose());
    // change the drive mode to Closed Loop
    swerveDriveBase.setDriveMode(DriveMode.CLOSED_LOOP);
  }), 
  swerveControllerCommand, 
  
  new InstantCommand(() -> {
    swerveDriveBase.stop(); 
    swerveDriveBase.setDriveMode(DriveMode.OPEN_LOOP);
  }));
  }

  private Command getSwerveAutoTest(){
    // Create config for trajectory
    TrajectoryConfig config =
    new TrajectoryConfig(
            Constants.SwerveBase.maxSpeed,
            Constants.SwerveBase.maxAcceleration)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(swerveDriveBase.getKinematics());

    //An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0, Units.feetToMeters(2 * Constants.VelocityConversions.ScaleFactor)), new Translation2d(0, Units.feetToMeters(3 * Constants.VelocityConversions.ScaleFactor))),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.feetToMeters(0), Units.feetToMeters(4 * Constants.VelocityConversions.ScaleFactor), new Rotation2d(0)),
            config);

    return runTrajecotory(exampleTrajectory); 
  } 

  private Command shootThenDrive(){
    TrajectoryConfig config = new TrajectoryConfig(Constants.SwerveBase.maxSpeed, Constants.SwerveBase.maxAcceleration)
                                    .setKinematics(swerveDriveBase.getKinematics());
      // An example trajectory to follow. All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(new Translation2d(Units.feetToMeters(-2.5 * Constants.VelocityConversions.ScaleFactor), 0), 
                                        new Translation2d(Units.feetToMeters(-5.0 * Constants.VelocityConversions.ScaleFactor), 0)),   
                                new Pose2d(Units.feetToMeters(5.0 * Constants.VelocityConversions.ScaleFactor), 0, new Rotation2d(0)),
                                config); 

    DoubleSupplier zeroSupplier = () -> { return 0.0; };
    DoubleSupplier speedLimitSupplier = () -> { return speedLimit; } ;

    return new TurnToAngle(zeroSupplier, zeroSupplier, speedLimitSupplier, this::angleToTarget, swerveDriveBase).withTimeout(5)
              .andThen( new ShootContinous(shooter,
                                           hood, 
                                           () -> { return DistanceToPower.calculate(distanceToTarget());}, 
                                           () -> { return DistanceToAngle.calculate(distanceToTarget());},
                                           limelight,
                                           true, 
                                           this::shooterOverrideEnabled).withTimeout(5) )
              .andThen( runTrajecotory(trajectory) );
  }

  private Command slalom(){ 
    String trajectoryJSON = "paths/Slalom.wpilib.json";
    Trajectory trajectory = new Trajectory(); 

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      trajectory.relativeTo(new Pose2d());
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace()); 
   } 

   return runTrajecotory(trajectory); 
  }  

  private Command waypointSlalom(){ 
  // Create config for trajectory
  TrajectoryConfig config =
  new TrajectoryConfig(
          Constants.SwerveBase.maxSpeed,
          Constants.SwerveBase.maxAcceleration)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(swerveDriveBase.getKinematics());

  // An example trajectory to follow. All units in meters.
  Trajectory trajectory =
      TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(
          new Translation2d(Units.feetToMeters(5 * Constants.VelocityConversions.ScaleFactor), 0), 
          new Translation2d(Units.feetToMeters(6 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(3 * Constants.VelocityConversions.ScaleFactor)), 
          new Translation2d(Units.feetToMeters(15 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(3 * Constants.VelocityConversions.ScaleFactor)), 
          new Translation2d(Units.feetToMeters(19 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(3 * Constants.VelocityConversions.ScaleFactor)), 
          new Translation2d(Units.feetToMeters(21 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(-2)), 
          new Translation2d(Units.feetToMeters(26 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(3 * Constants.VelocityConversions.ScaleFactor)), 
          new Translation2d(Units.feetToMeters(21 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(3 * Constants.VelocityConversions.ScaleFactor)), 
          new Translation2d(Units.feetToMeters(19 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(-2 * Constants.VelocityConversions.ScaleFactor)), 
          new Translation2d(Units.feetToMeters(15 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(-2 * Constants.VelocityConversions.ScaleFactor)), 
          new Translation2d(Units.feetToMeters(6 * Constants.VelocityConversions.ScaleFactor), 
          Units.feetToMeters(-1 * Constants.VelocityConversions.ScaleFactor)), 
          new Translation2d(Units.feetToMeters(5 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(3 * Constants.VelocityConversions.ScaleFactor))),   
          new Pose2d(Units.feetToMeters(1 * Constants.VelocityConversions.ScaleFactor), 
          Units.feetToMeters(3 * Constants.VelocityConversions.ScaleFactor), new Rotation2d(0)),
          config); 
 
           
  return runTrajecotory(trajectory); 


  } 


//Path A Red :)
  private Command galacticSearchPathARed(){ 
  TrajectoryConfig config =
  new TrajectoryConfig(
          Constants.SwerveBase.maxSpeed,
          Constants.SwerveBase.maxAcceleration)
      // Add kinematics to ensure max speed is actually obeyed 
      .setKinematics(swerveDriveBase.getKinematics());
  Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(
    new Translation2d(0, Units.feetToMeters(5 * Constants.VelocityConversions.ScaleFactor)), 
    new Translation2d(Units.feetToMeters(2.5 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(6 * Constants.VelocityConversions.ScaleFactor)), 
    new Translation2d(Units.feetToMeters(2.5 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(8 * Constants.VelocityConversions.ScaleFactor)), 
    new Translation2d(Units.feetToMeters(-3 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(11 * Constants.VelocityConversions.ScaleFactor)), 
    new Translation2d(Units.feetToMeters(-2 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(15 * Constants.VelocityConversions.ScaleFactor)), 
    new Translation2d(Units.feetToMeters(0 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(20 * Constants.VelocityConversions.ScaleFactor))   
    ), 
    new Pose2d(0, Units.feetToMeters(25 * Constants.VelocityConversions.ScaleFactor), new Rotation2d(0)), config);

 return new SequentialCommandGroup(new InstantCommand(intake::enable, intake), 
 runTrajecotory(trajectory), 
 new InstantCommand(intake::disable, intake)); 
//return runTrajecotory(trajectory); 
} 

//Path B Red :)
private Command galacticSearchPathBRed(){ 
  TrajectoryConfig config =
  new TrajectoryConfig(
          Constants.SwerveBase.maxSpeed,
          Constants.SwerveBase.maxAcceleration)
      // Add kinematics to ensure max speed is actually obeyed 
      .setKinematics(swerveDriveBase.getKinematics());
  Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(
    new Translation2d(Units.feetToMeters(0 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(4 * Constants.VelocityConversions.ScaleFactor)), 
    new Translation2d(Units.feetToMeters(4 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(6 * Constants.VelocityConversions.ScaleFactor)), 
    new Translation2d(Units.feetToMeters(5 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(8 * Constants.VelocityConversions.ScaleFactor)), 
    new Translation2d(Units.feetToMeters(3 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(10 * Constants.VelocityConversions.ScaleFactor)), 
    new Translation2d(Units.feetToMeters(-1 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(15 * Constants.VelocityConversions.ScaleFactor)),
    new Translation2d(Units.feetToMeters(0 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(20 * Constants.VelocityConversions.ScaleFactor))   
    ), 
    new Pose2d(Units.feetToMeters(0 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(25 * Constants.VelocityConversions.ScaleFactor), new Rotation2d(0)), config);

 return new SequentialCommandGroup(new InstantCommand(intake::enable, intake), 
 runTrajecotory(trajectory), 
 new InstantCommand(intake::disable, intake)); 
//return runTrajecotory(trajectory); 
} 

private Command bounce(){ 
  
    TrajectoryConfig config =
    new TrajectoryConfig(
            Constants.SwerveBase.maxSpeed,
            Constants.SwerveBase.maxAcceleration)
        // Add kinematics to ensure max speed is actually obeyed 
        .setKinematics(swerveDriveBase.getKinematics());
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(
      new Translation2d(Units.feetToMeters(0 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(3 * Constants.VelocityConversions.ScaleFactor)), 
      new Translation2d(Units.feetToMeters(-3.5 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(3 * Constants.VelocityConversions.ScaleFactor)), 
      new Translation2d(Units.feetToMeters(0 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(5 * Constants.VelocityConversions.ScaleFactor)), 
      new Translation2d(Units.feetToMeters(5 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(7 * Constants.VelocityConversions.ScaleFactor)), 
      //new Translation2d(Units.feetToMeters(0 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(5 * Constants.VelocityConversions.ScaleFactor)), 
      //new Translation2d(Units.feetToMeters(5 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(5 * Constants.VelocityConversions.ScaleFactor)),
      new Translation2d(Units.feetToMeters(4.5 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(11.5 * Constants.VelocityConversions.ScaleFactor)), 
      new Translation2d(Units.feetToMeters(-3 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(11.5 * Constants.VelocityConversions.ScaleFactor)), 
      new Translation2d(Units.feetToMeters(4.5 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(12.5 * Constants.VelocityConversions.ScaleFactor)),  
      new Translation2d(Units.feetToMeters(4.5 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(19.5 * Constants.VelocityConversions.ScaleFactor)), 
      new Translation2d(Units.feetToMeters(-3 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(19.5 * Constants.VelocityConversions.ScaleFactor)),
      new Translation2d(Units.feetToMeters(2 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(19.5 * Constants.VelocityConversions.ScaleFactor)),  
      new Translation2d(Units.feetToMeters(2 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(21 * Constants.VelocityConversions.ScaleFactor)) 
      ), 
      new Pose2d(Units.feetToMeters(2 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(23 * Constants.VelocityConversions.ScaleFactor), new Rotation2d(0)), config);
return runTrajecotory(trajectory); 

  } 

  public Command Barrel(){  
    TrajectoryConfig config =
    new TrajectoryConfig(
            Constants.SwerveBase.maxSpeed,
            Constants.SwerveBase.maxAcceleration)
        // Add kinematics to ensure max speed is actually obeyed 
        .setKinematics(swerveDriveBase.getKinematics());
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(
      new Translation2d(Units.feetToMeters(5 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(0 * Constants.VelocityConversions.ScaleFactor)), 
      new Translation2d(Units.feetToMeters(11 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(0 * Constants.VelocityConversions.ScaleFactor)), 
      new Translation2d(Units.feetToMeters(11 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(-5 * Constants.VelocityConversions.ScaleFactor)), 
      new Translation2d(Units.feetToMeters(5 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(-5 * Constants.VelocityConversions.ScaleFactor)), 
      new Translation2d(Units.feetToMeters(5 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(0 * Constants.VelocityConversions.ScaleFactor)), 
      new Translation2d(Units.feetToMeters(10 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(-1.5 * Constants.VelocityConversions.ScaleFactor)),
      new Translation2d(Units.feetToMeters(17 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(-1.5 * Constants.VelocityConversions.ScaleFactor)), 
      new Translation2d(Units.feetToMeters(17 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(2.5 * Constants.VelocityConversions.ScaleFactor)), 
      new Translation2d(Units.feetToMeters(12 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(2.5 * Constants.VelocityConversions.ScaleFactor)),  
      new Translation2d(Units.feetToMeters(15 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(-5 * Constants.VelocityConversions.ScaleFactor)), 
      new Translation2d(Units.feetToMeters(24 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(-6 * Constants.VelocityConversions.ScaleFactor)),
      new Translation2d(Units.feetToMeters(24 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(-6 * Constants.VelocityConversions.ScaleFactor)),  
      new Translation2d(Units.feetToMeters(24 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(-5 * Constants.VelocityConversions.ScaleFactor)), 
      new Translation2d(Units.feetToMeters(24 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(-1.5 * Constants.VelocityConversions.ScaleFactor)),    
      new Translation2d(Units.feetToMeters(10 * Constants.VelocityConversions.ScaleFactor ), Units.feetToMeters(-1.5 * Constants.VelocityConversions.ScaleFactor)) 
      ), 
      new Pose2d(Units.feetToMeters(0 * Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(0 * Constants.VelocityConversions.ScaleFactor), new Rotation2d(0)), config);
return runTrajecotory(trajectory);

  }
}