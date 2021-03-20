/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
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
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.commands.Autonomous1;
import frc.robot.commands.Autonomous2;
import frc.robot.commands.BatteryLED;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DriveByVelocity;
import frc.robot.commands.DriveToBall;
import frc.robot.commands.FireOnce;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.Indexer;
import frc.robot.commands.TestShooter;
import frc.robot.commands.TurnToBall;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.UnderGlow;
import frc.robot.commands.ZeroizeOdometry;
import frc.robot.commands.ZeroizeSwerveModules;
import frc.robot.lib.MathTools;
import frc.robot.lib.SmartSupplier;
import frc.robot.lib.thirdcoast.swerve.SwerveDrive.DriveMode;
import frc.robot.sensors.ColorSensor;
import frc.robot.sensors.FMSData;
import frc.robot.sensors.LEDStrip;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.ArcadeDrive;
import frc.robot.subsystems.SwerveDriveBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.SlewRateLimiter; 
import frc.robot.lib.SmartBooleanSupplier;

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
  private Limelight limelight = new Limelight();
  // public final Shooter launcher = new Shooter(limelight);
  // private Intake intake = new Intake();
  // private final Climber climber = new Climber();

  //private final ArcadeDrive driveBase = new ArcadeDrive();
  // private final ControlPanel controlPanel = new ControlPanel();
  // private final LidarLitePWM leftLidar = new LidarLitePWM(new DigitalInput(10));
  // private final LidarLitePWM rightLidar = new LidarLitePWM(new DigitalInput(11));
  // private final LidarReader lidarReader = new LidarReader();
  // private final ColorSensor colorSensor = new ColorSensor();

  private XboxController driver = new XboxController(Constants.pilot); 
  private XboxController operator = new XboxController(Constants.operator); 
  

  //private final Autonomous1 auto1 = new Autonomous1(driveBase);
  // private final Autonomous2 auto2 = new Autonomous2(driveBase, launcher, limelight);

  private LEDStrip ledStrip = new LEDStrip(Constants.PWM_Port.leds, Constants.PWM_Port.totalLEDCount);
  public final UnderGlow underGlow = new  UnderGlow(ledStrip);
  public final BatteryLED batteryMonitor = new BatteryLED(ledStrip);

  private SmartSupplier lowLeft = new SmartSupplier("Shooter/Low/Left", 10000);
  private SmartSupplier lowRight = new SmartSupplier("Shooter/Low/Right", 10000);
  private SmartSupplier midLeft = new SmartSupplier("Shooter/Mid/Left", 11000);
  private SmartSupplier midRight = new SmartSupplier("Shooter/Mid/Right", 11000);
  private SmartSupplier highLeft = new SmartSupplier("Shooter/High/Left", 16000);
  private SmartSupplier highRight = new SmartSupplier("Shooter/High/Right", 16000); 
  private double speedLimit = new SmartSupplier("Drivebase/SpeedLimit", 0.35).getAsDouble();
  public static double increase = 0; 

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    initShuffleBoard();
    limelight.setPipeline((byte) 0);
    limelight.setLED((byte) 1);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    // driveBase.setDefaultCommand( new DefaultDrive( this::driveSpeed, this::driveRotation, driveBase) );  

    boolean isLogitech = new SmartBooleanSupplier("Use Logitech Controller", false).getAsBoolean(); 

    swerveDriveBase.setDefaultCommand( new SwerveDrive( this::getForwardSwerve, 
    this::getStrafeSwerve, 
    isLogitech ? this::getYawSwerveLogitech : this::getYawSwerveXboxController, 
    isLogitech ? this::getSpeedLimitLogitech : this::getSpeedLimitXboxController,
    swerveDriveBase) ); 


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
    swerveDriveBase.stop();
    swerveDriveBase.zeroAzimuthEncoders();
    swerveDriveBase.zeroGyro();

  }

  //SlewRateLimiter driveSpeedFilter = new SlewRateLimiter(0.5);
  private double driveSpeed(){
    double speed = -driver.getY(Hand.kLeft);
    if(Math.abs(speed) < Constants.deadBand)
      speed = 0.0;
    return speed;
  }

  //SlewRateLimiter driveRotationFilter = new SlewRateLimiter(0.5);
  private double driveRotation(){
    double rotation = driver.getX(Hand.kRight);
    if(Math.abs(rotation) < Constants.deadBand)
      rotation = 0.0;
    // SmartDashboard.putNumber("Drive_Rotation", rotation);
    // return driveRotationFilter.calculate(rotation);
    return rotation;
  }

  public double getForwardSwerve() {
    return -driver.getY(Hand.kLeft); 
  }

  /** Left stick Y (left-right) axis. */
  public double getStrafeSwerve() {
    return driver.getX(Hand.kLeft);
  } 



  /** Right stick Y (left-right) axis. */
  public double getYawSwerveXboxController() {
    return driver.getX(Hand.kRight); 
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

  private double leftScissorPos(){
    double pos = operator.getY(Hand.kLeft);
    if (Math.abs(pos) < Constants.scissorDeadband)
      return 0.0;

    if(operator.getY(Hand.kLeft) < 0)
      pos = pos * pos;
    else
      pos = -(pos * pos);

      return MathTools.map(pos, -1.0, 1.0, -0.75, 0.75);
  }

  private double rightScissorPos(){
    double pos;
    if(operator.getY(Hand.kRight) < 0)
      pos = (operator.getY(Hand.kRight) * operator.getY(Hand.kRight));
    else
      pos = -(operator.getY(Hand.kRight) * operator.getY(Hand.kRight));
    if(Math.abs(pos) < 0.05)
      pos = 0.0;
    return MathTools.map(pos, -1.0, 1.0, -0.75, 0.75);
  } 



  SendableChooser<Integer> autoSelector; 
   
  private void initShuffleBoard(){
    SmartDashboard.putNumber("Shooter/Increase", increase);
    autoSelector = new SendableChooser<Integer>();
    autoSelector.addOption("Do nothing", 0);
    autoSelector.addOption("Drive forward.", 1);
    autoSelector.addOption("Shoot then drive", 2);
    autoSelector.addOption("Swerve Test Auto", 3);
    autoSelector.setDefaultOption("Do nothing", 0); 
    
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto") ;
    autoTab.add(autoSelector)
            .withWidget(BuiltInWidgets.kComboBoxChooser);
    SmartDashboard.putNumber("Auto/Selected_Auto", 1);
    autoTab.add("Auto/Initial_Delay", 0); 


  
    if(!Robot.fmsAttached){
      ShuffleboardTab testCommandsTab = Shuffleboard.getTab("Test Commands"); 
      // testCommandsTab.add( new TestShooter(launcher, driver::getPOV));
      // testCommandsTab.add( new FireOnce(launcher)
      //                 .andThen(new InstantCommand( () -> {launcher.stopFlywheels(); launcher.stopIndexer();}) )); 
      testCommandsTab.add(new ZeroizeSwerveModules(swerveDriveBase)); 
      testCommandsTab.add(new ZeroizeOdometry(swerveDriveBase));
      // testCommandsTab.add( new DriveByVelocity(driveBase));

      RunCommand getColor = new RunCommand( FMSData::getColor );
      getColor.setName("Get_Color");
      testCommandsTab.add(getColor);
  
      testCommandsTab.add( batteryMonitor );
      testCommandsTab.add( underGlow );
     
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

      // InstantCommand resetScizzorsCommand = new InstantCommand(climber::resetClimberEncoders);
      // resetScizzorsCommand.setName("Reset Scizzors");
      // testCommandsTab.add(resetScizzorsCommand);

      // InstantCommand toggleClimbLimits = new InstantCommand(climber::toggleLimits);
      // toggleClimbLimits.setName("Toggle Climb Limits");
      // testCommandsTab.add(toggleClimbLimits);
    }

    //  SmartDashboard.putData( new PowerDistributionPanel(Constants.PDP) );
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /**
     * Driver:
     * +Left joystick: drive 
     * +Right joystick: turn
     * +B: drive backwards
     * +Right bumper: intake
     * +Left bumper: outtake
     * +X: toggle intake elevation
     * +D-pad up: increase shooter power
     * +D-pad down: decrease shooter power
     * +A: toggle Drive to ball
     * 
     * Operator:
     * +Right bumper: run indexer
     * +Left bumper: shoot
     * +Y: prep flywheels auto
     * +B: prep flywheels close
     * 
     * +A: prep flywheels far
     * +X: stop flywheels
     * +Left trigger: manual turret adjust left
     * +Right trigger: manual turret adjust right
     * +D-pad left: toggle limelight pipeline
     * +D-pad right toggle shooter elevation
     * +D-pad up
     * : scissors up
     * +D-pad down: scissors down
     * +Left joystick: left scissor
     * +Right joystick: right scissor
     * +Start: scissors enable
     */

     //B: drive backwards
    // new JoystickButton(driver, XboxController.Button.kB.value)
    //   .whenPressed( new InstantCommand( driveBase::toggleDriveDirection, driveBase) ); 

    // //Right bumper: intake
    // new JoystickButton(driver, XboxController.Button.kBumperRight.value)   
    //   .whenHeld( new InstantCommand( intake :: enable))      
    //   .whenReleased( new InstantCommand(intake :: disable));    
    
    //Left bumper: outtake 
    // new JoystickButton(driver, XboxController.Button.kBumperLeft.value)
    //   .whenHeld( new InstantCommand( intake::outtake))
    //   .whenHeld( new InstantCommand( launcher::outtake))
    //   .whenReleased( new InstantCommand(intake::disable))
    //   .whenReleased( new InstantCommand(launcher::stopIndexer));

    //X toggle intake elevation
    // new JoystickButton(driver, XboxController.Button.kX.value)
    //   .whenPressed( new InstantCommand(() -> {intake.stop(); intake.toggleElevation(); }));

    //D-pad up: increase power
    new POVButton(driver, 0)
      .whenPressed( new InstantCommand( () -> {increase += 250; 
                                              SmartDashboard.putNumber("Shooter/Increase", increase);
                                            } ));

    //D-pad down: decrease power
    new POVButton(driver, 0)
      .whenPressed( new InstantCommand( () -> {increase -= 250;
                                              SmartDashboard.putNumber("Shooter/Increase", increase);
                                            } ));

    // new JoystickButton(driver, XboxController.Button.kA.value)
    //   .whenHeld(new SequentialCommandGroup(new TurnToBall(driveBase, limelight, 0.1),
    //             new ParallelCommandGroup(new InstantCommand(intake::enable), new DriveToBall(limelight, driveBase)),
    //             new ParallelCommandGroup(new InstantCommand(intake::disable), new Indexer(launcher).withTimeout(0.5), new TurnToBall(driveBase, limelight, 0.1)),
    //             new ParallelCommandGroup(new InstantCommand(intake::enable), new DriveToBall(limelight, driveBase))
    //   ))
    //   .whenReleased(new ParallelCommandGroup(new InstantCommand(intake::disable), new InstantCommand(launcher::stopIndexer)));

    new JoystickButton(driver, XboxController.Button.kB.value) 
        .whenHeld(new SequentialCommandGroup(
                        new TurnToBall(swerveDriveBase, limelight, 0.1), 
                        new DriveToBall(swerveDriveBase, limelight)));
    //Left bumper: fire auto
    // new JoystickButton(operator, XboxController.Button.kBumperLeft.value)
    //   .whenHeld( new FireOnce(launcher) )
    //   .whenReleased(launcher::stopIndexer);

    //Right bumper: run indexer
    // new JoystickButton(operator, XboxController.Button.kBumperRight.value)
    //   .whenPressed(launcher::fire)
    //   .whenReleased(launcher::stopIndexer);

    //Y: prep flywheels auto
    // new JoystickButton(operator, XboxController.Button.kY.value)
    //   .whenPressed( new InstantCommand( () -> {launcher.prepFlywheels(lowLeft, lowRight);}))
    //   .whenPressed( new TurnToTarget(launcher, driveBase, limelight));

    //B: prep flywheels close
    // new JoystickButton(operator, XboxController.Button.kB.value)
    //   .whenPressed( new InstantCommand( () -> {launcher.prepFlywheels(midLeft, midRight);}))
    //   .whenPressed( new TurnToTarget(launcher, driveBase, limelight));

    //A: prep flywheels far
    // new JoystickButton(operator, XboxController.Button.kA.value)
    //   .whenPressed( new InstantCommand( () -> {launcher.prepFlywheels(highLeft, highRight);}))
    //   .whenPressed( new TurnToTarget(launcher, driveBase, limelight));

    //X: stop flywheels
    // new JoystickButton(operator, XboxController.Button.kX.value)
    //   .whenPressed( new InstantCommand(launcher::stopFlywheels))
    //   .whenPressed( new InstantCommand(launcher::goHome));

    //Left trigger: manual turret adjust left
    // new Trigger(() -> operator.getTriggerAxis(Hand.kLeft) > 0.1 )
    // .whileActiveOnce( new FunctionalCommand(() -> {launcher.tracking = false;},
    //                                         () -> {launcher.turnTurret((int) MathTools.map(operator.getTriggerAxis(Hand.kLeft), 0.1, 1, 1, 10));},
    //                                         (interrupted) -> {launcher.stopTurret();},
    //                                         launcher::turretTurnIsComplete,
    //                                         launcher) );
    
  //Right trigger: manual turret adjust right
  // new Trigger(() -> operator.getTriggerAxis(Hand.kRight) > 0.1 )
  // .whileActiveOnce( new FunctionalCommand(() -> {launcher.tracking = false;},
  //                                         () -> {launcher.turnTurret(-(int) MathTools.map(operator.getTriggerAxis(Hand.kRight), 0.1, 1, 1, 10));},
  //                                         (interrupted) -> {launcher.stopTurret();},
  //                                         launcher::turretTurnIsComplete,
  //                                         launcher) );

    // //Left trigger: rotation control
    // new Trigger(() -> operator.getTriggerAxis(Hand.kLeft) > 0.1 )
    //   .whileActiveOnce( new FunctionalCommand( () -> controlPanel.rotationControl(Constants.ControlPanelConstants.ROTATION_CONTROL_DISTANCE),
    //                                         () -> {},
    //                                         (interrupted) -> { controlPanel.stop(); }, 
    //                                         controlPanel::isRotationComplete,
    //                                         controlPanel ) );

    // //Right trigger: position control
    // new Trigger(() -> operator.getTriggerAxis(Hand.kRight) > 0.1 )
    //   .whileActiveOnce(  new FunctionalCommand(controlPanel::positionControl,
    //                                       () -> {},
    //                                       (interrupted) -> { controlPanel.stop(); },
    //                                       controlPanel::checkColor,
    //                                       controlPanel
    //                                       )
    //                 .andThen( new FunctionalCommand( () -> controlPanel.rotationControl(Constants.ControlPanelConstants.COLOR_ADJUST),
    //                                       () -> {},
    //                                       (interrupted) -> { controlPanel.stop(); },
    //                                       controlPanel::isRotationComplete,
    //                                       controlPanel) ));

    // //D-pad left: control panel left
    // new POVButton(operator, 270)
    // .whenPressed(  new FunctionalCommand( () -> controlPanel.rotationControl(-Constants.ControlPanelConstants.POSITION_ADJUST),
    //                                     () -> {},
    //                                     (interrupted) -> { controlPanel.stop(); },
    //                                     controlPanel::isRotationComplete,
    //                                     controlPanel));   

    // //D-pad right: control panel right
    // new POVButton(operator, 90)
    // .whenPressed(  new FunctionalCommand( () -> controlPanel.rotationControl(Constants.ControlPanelConstants.POSITION_ADJUST),
    //                                       () -> {},
    //                                       (interrupted) -> { controlPanel.stop(); },
    //                                       controlPanel::isRotationComplete,
    //                                       controlPanel)); 

    //Back: toggle shooter elevation
    // new JoystickButton(operator, XboxController.Button.kBack.value)
    // .whenPressed(new InstantCommand( launcher::toggleElevation));

    //D-pad up: scissors up
    // new POVButton(operator, 0)
    //   .whenPressed( new FunctionalCommand( () -> climber.reachUp(),
    //                                        () -> {},
    //                                        (interupted) -> {},
    //                                        climber::climbInPosition,
    //                                        climber).withTimeout(5) );

    //D-pad left: scissors to 45"
    // new POVButton(operator, 270)
    //   .whenPressed( new FunctionalCommand( () -> climber.reachLow(),
    //                                        () -> {},
    //                                        (interupted) -> {},
    //                                        climber::climbInPosition,
    //                                        climber).withTimeout(5) );

    //D-pad down: scissors down
    // new POVButton(operator, 180)
    //   .whenPressed( new FunctionalCommand( () -> climber.goHome(),
    //                                        () -> {},
    //                                        (interupted) -> {},
    //                                        climber::climbInPosition,
    //                                        climber).withTimeout(5) );

    //Joysticks: manual scissor control
    // new Trigger(() -> (Math.abs(operator.getY(Hand.kLeft)) > Constants.scissorDeadband) 
    //                  || (Math.abs(operator.getY(Hand.kRight)) > Constants.scissorDeadband))
    //   .whenActive(new RunCommand(() -> {climber.directControl(leftScissorPos(), rightScissorPos());}, climber ));

    //Start: enable scissors
    // new JoystickButton(operator, XboxController.Button.kStart.value)
    //   .whenPressed( new InstantCommand( climber::toggleEnable));

                                          
    // DoubleSupplier setPoint = new DoubleSupplier(){
    //   public double getAsDouble() {
    //     return SmartDashboard.getNumber("TurnPID/setPoint", 0.0)+driveBase.getHeading();
    //   }
    // };

    // SmartDashboard.putNumber("TurnPID/offset", 15);
    // new JoystickButton(operator, XboxController.Button.kStart.value)
    //   .whenPressed( new InstantCommand( () -> {
    //     SmartDashboard.putNumber("TurnPID/setPoint", driveBase.getHeading()+SmartDashboard.getNumber("turnPID/offset", 15));
    //   }) )
    //   .whenHeld( new TurnToAngle(driveBase, () -> SmartDashboard.getNumber("TurnPID/setPoint", 0.0) ).beforeStarting( () -> {
    //                               Constants.TurnPID.kP = SmartDashboard.getNumber("TurnPID/kP",Constants.TurnPID.kP) ;
    //                               Constants.TurnPID.kI = SmartDashboard.getNumber("TurnPID/kI",Constants.TurnPID.kI) ;
    //                               Constants.TurnPID.kD = SmartDashboard.getNumber("TurnPID/kD",Constants.TurnPID.kD) ;
    //                             })
    //   );

      // new JoystickButton(driver, XboxController.Button.kStart.value)
      //   .whenHeld( new FollowPath( PathGenerator.driveTestOne(), driveBase).andThen( () -> driveBase.stop() ) )
      //   .whenReleased( () -> driveBase.stop()); 

      

    //   new JoystickButton(driver, XboxController.Button.kA.value) 
    //   .whenHeld(new DriveByVelocity(driveBase));      
    
      // new JoystickButton(driver, XboxController.Button.kBack.value).whenPressed(new InstantCommand(driveBase::resetPosition));
  };


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
              // Map.entry(1, auto1),
              // Map.entry(2, auto2),
              Map.entry(3, getSwerveAutoTest()) 
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

  private Command getSwerveAutoTest(){
    if (swerveDriveBase == null){
      return new PrintCommand("Wrong drivebase -- expecting swerve");
    } 

    // Create config for trajectory
    TrajectoryConfig config =
    new TrajectoryConfig(
            Constants.SwerveBase.maxSpeed,
            Constants.SwerveBase.maxAcceleration)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(swerveDriveBase.getKinematics());

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            new SmartSupplier("Swerve rP", 0).getAsDouble(),
            new SmartSupplier("Swerve rI", 0).getAsDouble(), 
            new SmartSupplier("Swerve rD", 0).getAsDouble(), 
            new TrapezoidProfile.Constraints(Constants.SwerveBase.maxAngularSpeed, Constants.SwerveBase.maxAngularAccelerartion));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            swerveDriveBase::getPose, // Functional interface to feed supplier
            swerveDriveBase.getKinematics(),

            // Position controllers
            new PIDController(  new SmartSupplier("Swerve xP", 0).getAsDouble(),
                                new SmartSupplier("Swerve xI", 0).getAsDouble(), 
                                new SmartSupplier("Swerve xD", 0).getAsDouble()),
            new PIDController(  new SmartSupplier("Swerve yP", 0).getAsDouble(),
                                new SmartSupplier("Swerve yI", 0).getAsDouble(), 
                                new SmartSupplier("Swerve yD", 0).getAsDouble()),
            thetaController,
            swerveDriveBase::setModuleStates,
            swerveDriveBase);

    // Reset odometry to the starting pose of the trajectory.
    swerveDriveBase.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return new SequentialCommandGroup( 
      new InstantCommand(() -> {
        swerveDriveBase.setDriveMode(DriveMode.CLOSED_LOOP);
      }), 
      swerveControllerCommand, 
      new InstantCommand(() -> {
        swerveDriveBase.stop(); 
        swerveDriveBase.setDriveMode(DriveMode.OPEN_LOOP);
      }));
  }
}
