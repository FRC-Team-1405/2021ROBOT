package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.thirdcoast.swerve.SwerveDrive;
import frc.robot.lib.thirdcoast.swerve.SwerveDrive.DriveMode;
import frc.robot.lib.thirdcoast.swerve.SwerveDriveConfig;
import frc.robot.lib.thirdcoast.swerve.Wheel;

public class SwerveDriveBase extends SubsystemBase {

  private static final double DRIVE_SETPOINT_MAX = 1.0;
  private static final double ROBOT_LENGTH = 1.0;
  private static final double ROBOT_WIDTH = 1.0;
  private final SwerveDrive swerve = getSwerve();

  public SwerveDriveBase() {
  }

  public void setDriveMode(DriveMode mode) {
    swerve.setDriveMode(mode);
  }

  public void stop() {
    swerve.stop();
  }

  public void zeroAzimuthEncoders() {
    swerve.zeroAzimuthEncoders();
  } 

  public void writeAzimuthPositions(){ 
    swerve.saveAzimuthPositions();
  }

  public void drive(double forward, double strafe, double azimuth, double speedLimit) {
    swerve.drive(forward, strafe, azimuth, speedLimit);
   } 

  // Drive the robot in a straight line without using navX
  public void driveStraight(double speed, double turnspeed, double speedLimit){
    swerve.setFieldOriented(false);
    swerve.drive(speed, 0, turnspeed, speedLimit);
    swerve.setFieldOriented(true);
  }

  public void zeroGyro() {
    AHRS gyro = swerve.getGyro();
    gyro.setAngleAdjustment(0);
    double adj = (-gyro.getAngle() % 360) + SwerveDriveConfig.gyroHardwareOffset;
    gyro.setAngleAdjustment(-adj);
  } 

  // Swerve configuration

  private SwerveDrive getSwerve() {
    SwerveDriveConfig config = new SwerveDriveConfig();
    config.wheels = getWheels();
    config.gyro = new AHRS(SPI.Port.kMXP);
    config.length = ROBOT_LENGTH;
    config.width = ROBOT_WIDTH;
    config.gyroLoggingEnabled = true;
    config.summarizeTalonErrors = false;

    return new SwerveDrive(config);
  }

  private Wheel[] getWheels() {
    // wheels should be created in the array as
    // [0] -> front_right
    // [1] -> front_left
    // [2] -> rear_left
    // [3] -> rear_right
    Wheel[] wheels = new Wheel[] {
      createWheel(Constants.SwerveBase.azimuthFrontRight, Constants.SwerveBase.driveFrontRight, Constants.SwerveBase.encoderFrontRight, Constants.SwerveBase.frontRightLocation),
      createWheel(Constants.SwerveBase.azimuthFrontLeft, Constants.SwerveBase.driveFrontLeft, Constants.SwerveBase.encoderFrontLeft, Constants.SwerveBase.frontLeftLocation),
      createWheel(Constants.SwerveBase.azimuthBackLeft, Constants.SwerveBase.driveBackLeft, Constants.SwerveBase.encoderBackLeft, Constants.SwerveBase.backLeftLocation),
      createWheel(Constants.SwerveBase.azimuthBackRight, Constants.SwerveBase.driveBackRight, Constants.SwerveBase.encoderBackRight, Constants.SwerveBase.backRightLocation)
    };
    
    return wheels;
  }

  private Wheel createWheel(int azimuthId, int driveId, int encoderId, Translation2d position){
    TalonFX azimuthTalon = new TalonFX(azimuthId);
    TalonFX driveTalon = new TalonFX(driveId);
    CANCoder encoder = new CANCoder(encoderId);

    TalonFXConfiguration allConfigs = new TalonFXConfiguration();
    azimuthTalon.getAllConfigs(allConfigs);
    allConfigs.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    allConfigs.remoteFilter0.remoteSensorDeviceID = encoder.getDeviceID();
    azimuthTalon.configAllSettings(allConfigs);

    return new Wheel(azimuthTalon, driveTalon, encoder, position, DRIVE_SETPOINT_MAX);
  }  

  @Override
  public void periodic() {  
    swerve.updateOdometry(); 
  }

  public void zeroizeOdometry(){
    swerve.resetOdometry(); 
    } 

  public void resetOdometry(Pose2d pose){
      swerve.resetOdometry(pose); 
      } 
  
    public SwerveDriveKinematics getKinematics(){ 
   return swerve.getKinematics(); 
  } 

  public void setModuleStates(SwerveModuleState[] states){ 
    swerve.setModuleStates(states);
  } 

  public Pose2d getPose(){ 
   return swerve.getPose();
  }
}