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
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
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
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Hood;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveToBall;
import frc.robot.commands.PrepareShooter;
import frc.robot.commands.ShootConstantly;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.TurnToBall;
import frc.robot.commands.ZeroizeOdometry;
import frc.robot.commands.ZeroizeSwerveModules;
import frc.robot.lib.CustomPIDController;
import frc.robot.lib.Interpolate;
import frc.robot.lib.MathTools;
import frc.robot.lib.SmartBooleanSupplier;
import frc.robot.lib.SmartSupplier;
import frc.robot.lib.thirdcoast.swerve.SwerveDrive.DriveMode;
import frc.robot.sensors.LidarLitePWM;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDriveBase;
import frc.robot.subsystems.SwerveIntake;

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
  public final SwerveIntake intake = new SwerveIntake(); 
  private Limelight limelight = new Limelight();

  private XboxController driver = new XboxController(Constants.pilot); 
  private XboxController operator = new XboxController(Constants.operator); 
  private XboxController demoController = new XboxController(Constants.demoController);
  
  private Shooter shooter = new Shooter(); 
  private Hood hood = new Hood(); 
  private LidarLitePWM aimingLidar = new LidarLitePWM(new DigitalInput(9));


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
    (() -> { return swerveDriveBase.getPose().getRotation().getDegrees() ; }),
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

  SendableChooser<Integer> autoSelector; 
   
  private void initShuffleBoard(){
    SmartDashboard.putNumber("Shooter/Increase", increase);
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


  
    if(!Robot.fmsAttached){
      ShuffleboardTab testCommandsTab = Shuffleboard.getTab("Test Commands"); 
      // testCommandsTab.add( new TestShooter(launcher, driver::getPOV));
      // testCommandsTab.add( new FireOnce(launcher)
      //                 .andThen(new InstantCommand( () -> {launcher.stopFlywheels(); launcher.stopIndexer();}) )); 
      testCommandsTab.add(new ZeroizeSwerveModules(swerveDriveBase)); 
      testCommandsTab.add(new ZeroizeOdometry(swerveDriveBase));
      // testCommandsTab.add( new DriveByVelocity(driveBase));

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

    new JoystickButton(driver, XboxController.Button.kB.value) 
        .whenHeld(new SequentialCommandGroup(
                        new TurnToBall(swerveDriveBase, limelight, 0.1), 
                        new DriveToBall(swerveDriveBase, limelight))); 

    new JoystickButton(driver, XboxController.Button.kA.value).whenPressed(new InstantCommand(shooter::testShoot, shooter)).whenReleased(shooter::stop, shooter);                     

    //new JoystickButton(driver, XboxController.Button.kY.value).whenHeld(new ShootConstantly(shooter, hood, aimingLidar));                     

    new JoystickButton(driver, XboxController.Button.kBumperLeft.value)
      .whenPressed(new SequentialCommandGroup(new InstantCommand(shooter::index), new WaitCommand(.5), new InstantCommand(shooter::close)));    

    new JoystickButton(driver, XboxController.Button.kBumperRight.value) 
      .whenPressed(new InstantCommand(intake::intake, intake)) 
      .whenReleased(new InstantCommand(intake::stop, intake));
       
    configureDemoController();
  };

  private void configureDemoController(){ 

    SmartDashboard.putNumber("shooter speed", 0.0); 
    SmartDashboard.putNumber("shooter angle", 0.0);

    new JoystickButton(demoController, XboxController.Button.kY.value).whileHeld(new InstantCommand(() ->{
      Interpolate distanceToAngle = new Interpolate("ToDo");
      hood.setPosition( (int) distanceToAngle.CalculateOutput(aimingLidar.getDistance()));
    })); 
    
    new JoystickButton(demoController, XboxController.Button.kA.value)
            .whileHeld( new PrepareShooter( shooter, 
                                            hood, 
                                            () -> { return SmartDashboard.getNumber("shooter speed", 0.0);}, 
                                            () -> { return SmartDashboard.getNumber("shooter angle", 0.0);},
                                            () -> { return aimingLidar.getDistance();}
                                            )); 
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
              //Map.entry(2, auto2),
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

  private Command circle(){ 
    TrajectoryConfig config = new TrajectoryConfig(
      Constants.SwerveBase.maxSpeed,
      Constants.SwerveBase.maxAcceleration)
  // Add kinematics to ensure max speed is actually obeyed
  .setKinematics(swerveDriveBase.getKinematics()); 

  Trajectory circle = TrajectoryGenerator.generateTrajectory( new Pose2d(0, 0, new Rotation2d(0)), 
  List.of(new Translation2d(Units.feetToMeters(0 * Constants.VelocityConversions.ScaleFactor), 0 * Units.feetToMeters(Constants.VelocityConversions.ScaleFactor)), 
          new Translation2d(Units.feetToMeters(4 * Constants.VelocityConversions.ScaleFactor), 4 * Constants.VelocityConversions.ScaleFactor), 
          new Translation2d(Units.feetToMeters(8 * Constants.VelocityConversions.ScaleFactor), 0 * Units.feetToMeters(Constants.VelocityConversions.ScaleFactor)), 
          new Translation2d(Units.feetToMeters(-4* Constants.VelocityConversions.ScaleFactor), Units.feetToMeters(-4* Constants.VelocityConversions.ScaleFactor))), 
          new Pose2d(0, 0, new Rotation2d(0)), config);  
   

  return runTrajecotory(circle); }

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

 return new SequentialCommandGroup(new InstantCommand(intake::intake, intake), 
 runTrajecotory(trajectory), 
 new InstantCommand(intake::stop, intake)); 
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

 return new SequentialCommandGroup(new InstantCommand(intake::intake, intake), 
 runTrajecotory(trajectory), 
 new InstantCommand(intake::stop, intake)); 
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