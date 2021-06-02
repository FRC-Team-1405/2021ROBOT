package frc.robot.lib.thirdcoast.swerve;

import javax.swing.text.html.HTMLDocument.HTMLReader.PreAction;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

import java.util.logging.LogManager;
import java.util.logging.Logger;

import frc.robot.Constants;
import frc.robot.lib.thirdcoast.talon.Errors;

/**
 * Control a Third Coast swerve drive.
 *
 * <p>Wheels are a array numbered 0-3 from front to back, with even numbers on the left side when
 * facing forward.
 *
 * <p>Derivation of inverse kinematic equations are from Ether's <a
 * href="https://www.chiefdelphi.com/media/papers/2426">Swerve Kinematics and Programming</a>.
 *
 * @see Wheel
 */
@SuppressWarnings("unused")
public class SwerveDrive {

  public static final int DEFAULT_ABSOLUTE_AZIMUTH_OFFSET = 200;
  private static final Logger logger = Logger.getLogger(SwerveDrive.class.getName());
  private static final int WHEEL_COUNT = 4;
  private final AHRS gyro;
  private final double kLengthComponent;
  private final double kWidthComponent;
  private final double kGyroRateCorrection;
  private final Wheel[] wheels;
  private final double[] ws = new double[WHEEL_COUNT];
  private final double[] wa = new double[WHEEL_COUNT];
  private boolean isFieldOriented; 
  private SwerveDriveKinematics kinematics; 
  
  SwerveDriveOdometry m_odometry; 

  public SwerveDrive(SwerveDriveConfig config) {
    gyro = config.gyro;
    wheels = config.wheels;

    Preferences prefs = Preferences.getInstance();
    for (int i = 0; i < WHEEL_COUNT; i++) {
      String key = getPreferenceKeyForWheel(i) ;
      if (!prefs.containsKey(key)) {
        prefs.putInt(key, DEFAULT_ABSOLUTE_AZIMUTH_OFFSET);
      }
    }
  
    final boolean summarizeErrors = config.summarizeTalonErrors;
    Errors.setSummarized(summarizeErrors);
    Errors.setCount(0);
    logger.warning(String.format("TalonSRX configuration errors summarized = %b", summarizeErrors));

    double length = config.length;
    double width = config.width;
    double radius = Math.hypot(length, width);
    kLengthComponent = length / radius;
    kWidthComponent = width / radius;

    logger.info(String.format("gyro is configured: %b", gyro != null));
    logger.info(String.format("gyro is connected: %b", gyro != null && gyro.isConnected()));
    setFieldOriented(gyro != null && gyro.isConnected());

    if (isFieldOriented) {
      gyro.enableLogging(config.gyroLoggingEnabled);
      double robotPeriod = config.robotPeriod;
      double gyroRateCoeff = config.gyroRateCoeff;
      int rate = gyro.getActualUpdateRate();
      double gyroPeriod = 1.0 / rate;
      kGyroRateCorrection = (robotPeriod / gyroPeriod) * gyroRateCoeff;
      logger.config(String.format("gyro frequency = %d Hz", rate));
    } else {
      logger.warning("gyro is missing or not enabled");
      kGyroRateCorrection = 0;
    }


    kinematics = new SwerveDriveKinematics(wheels[0].getPostion(), wheels[1].getPostion(), wheels[2].getPostion(), wheels[3].getPostion());

    m_odometry = new SwerveDriveOdometry(kinematics, getGyroAngle());

    logger.config(String.format("length = %f", length));
    logger.config(String.format("width = %f", width));
    logger.config(String.format("enableGyroLogging = %b", config.gyroLoggingEnabled));
    logger.config(String.format("gyroRateCorrection = %f", kGyroRateCorrection));
  }

  public SwerveDriveKinematics getKinematics(){ 
    return kinematics; 
  } 

  /**
   * Return key that wheel zero information is stored under in WPI preferences.
   *
   * @param wheel the wheel number
   * @return the String key
   */
  
   private static final String[] WHEELNAME = {"FL", "FR", "BL", "BR" };

  public static String getPreferenceKeyForWheel(int wheel) {
    return String.format("%s/wheel.%s", SwerveDrive.class.getSimpleName(), WHEELNAME[wheel]);

  }

  /**
   * Set the drive mode.
   *
   * @param driveMode the drive mode
   */
  public void setDriveMode(DriveMode driveMode) {
    for (Wheel wheel : wheels) {
      wheel.setDriveMode(driveMode);
    }
    logger.config(String.format("drive mode = %s", driveMode));
  }

  /**
   * Set all four wheels to specified values.
   *
   * @param azimuth -0.5 to 0.5 rotations, measured clockwise with zero being the robot
   *     straight-ahead position
   * @param drive 0 to 1 in the direction of the wheel azimuth
   */
  public void set(double azimuth, double drive) {
    for (Wheel wheel : wheels) {
      wheel.set(azimuth, drive);
    }
  }

  /**
   * Drive the robot in given field-relative direction and with given rotation.
   *
   * @param forward Y-axis movement, from -1.0 (reverse) to 1.0 (forward)
   * @param strafe X-axis movement, from -1.0 (left) to 1.0 (right)
   * @param azimuth robot rotation, from -1.0 (CCW) to 1.0 (CW)
   */
  public void drive(double forward, double strafe, double azimuth, double speedLimit) {

    // Use gyro for field-oriented drive. We use getAngle instead of getYaw to enable arbitrary
    // autonomous starting positions.
    if (isFieldOriented) {
      double angle = -gyro.getAngle(); 
      angle += gyro.getRate() * kGyroRateCorrection;
      angle = Math.IEEEremainder(angle, 360.0);

      angle = Math.toRadians(angle);
      final double temp = forward * Math.cos(angle) + strafe * Math.sin(angle);
      strafe = strafe * Math.cos(angle) - forward * Math.sin(angle);
      forward = temp;
    } 

    final double a = strafe - azimuth * kLengthComponent;
    final double b = strafe + azimuth * kLengthComponent;
    final double c = forward - azimuth * kWidthComponent;
    final double d = forward + azimuth * kWidthComponent;

    // wheel speed
    ws[0] = Math.hypot(b, d)  * speedLimit;
    ws[1] = Math.hypot(b, c)  * speedLimit;
    ws[2] = Math.hypot(a, d)  * speedLimit;
    ws[3] = Math.hypot(a, c)  * speedLimit;

    // wheel azimuth
    wa[0] = Math.atan2(b, d) * 0.5 / Math.PI;
    wa[1] = Math.atan2(b, c) * 0.5 / Math.PI;
    wa[2] = Math.atan2(a, d) * 0.5 / Math.PI;
    wa[3] = Math.atan2(a, c) * 0.5 / Math.PI;

    // normalize wheel speed
    final double maxWheelSpeed = Math.max(Math.max(ws[0], ws[1]), Math.max(ws[2], ws[3]));
    if (maxWheelSpeed > 1.0) {
      for (int i = 0; i < WHEEL_COUNT; i++) {
        ws[i] /= maxWheelSpeed;
      }
    }

    logger.finer( () -> String.format("Angle FL(%f) FR(%f) BL(%f) BR(%f)", wa[1], wa[0], wa[2], wa[3]));
    // set wheels
    for (int i = 0; i < WHEEL_COUNT; i++) {
      wheels[i].set(wa[i], ws[i]);
    }
  }

  /**
   * Stops all wheels' azimuth and drive movement. Calling this in the robots {@code teleopInit} and
   * {@code autonomousInit} will reset wheel azimuth relative encoders to the current position and
   * thereby prevent wheel rotation if the wheels were moved manually while the robot was disabled.
   */
  public void stop() {
    for (Wheel wheel : wheels) {
      wheel.stop();
    }
    logger.info("stopped all wheels");
  }

  /**
   * Save the wheels' azimuth current position as read by absolute encoder. These values are saved
   * persistently on the roboRIO and are normally used to calculate the relative encoder offset
   * during wheel initialization.
   *
   * <p>The wheel alignment data is saved in the WPI preferences data store and may be viewed using
   * a network tables viewer.
   *
   * @see #zeroAzimuthEncoders()
   */
  public void saveAzimuthPositions() {
    saveAzimuthPositions(Preferences.getInstance());
  }

  void saveAzimuthPositions(Preferences prefs) {
    for (int i = 0; i < WHEEL_COUNT; i++) {
      int position = wheels[i].getAzimuthAbsolutePosition();
      prefs.putInt(getPreferenceKeyForWheel(i), position);
      logger.info(String.format("azimuth %d: saved zero = %d", i, position));
    }
  }

  /**
   * Set wheels' azimuth relative offset from zero based on the current absolute position. This uses
   * the physical zero position as read by the absolute encoder and saved during the wheel alignment
   * process.
   *
   * @see #saveAzimuthPositions()
   */
  public void zeroAzimuthEncoders() {
    zeroAzimuthEncoders(Preferences.getInstance());
  }

  void zeroAzimuthEncoders(Preferences prefs) {
    Errors.setCount(0);
    for (int i = 0; i < WHEEL_COUNT; i++) {
      int position = prefs.getInt(getPreferenceKeyForWheel(i), DEFAULT_ABSOLUTE_AZIMUTH_OFFSET);
      wheels[i].setAzimuthZero(position);
      logger.info(String.format("azimuth %d: loaded zero = %d", i, position));
    }
    int errorCount = Errors.getCount();
    if (errorCount > 0) logger.warning(String.format("TalonSRX set azimuth zero error count = {}", errorCount));
  }

  /**
   * Returns the four wheels of the swerve drive.
   *
   * @return the Wheel array.
   */
  public Wheel[] getWheels() {
    return wheels;
  }

  /**
   * Get the gyro instance being used by the drive.
   *
   * @return the gyro instance
   */
  public AHRS getGyro() {
    return gyro;
  }

  /**
   * Get status of field-oriented driving.
   *
   * @return status of field-oriented driving.
   */
  public boolean isFieldOriented() {
    return isFieldOriented;
  }

  /**
   * Enable or disable field-oriented driving. Enabled by default if connected gyro is passed in via
   * {@code SwerveDriveConfig} during construction.
   *
   * @param enabled true to enable field-oriented driving.
   */
  public void setFieldOriented(boolean enabled) {
    isFieldOriented = enabled;
    logger.info(String.format("field orientation driving is %s", isFieldOriented ? "ENABLED" : "DISABLED"));
  }

  /**
   * Unit testing
   *
   * @return length
   */
  double getLengthComponent() {
    return kLengthComponent;
  }

  /**
   * Unit testing
   *
   * @return width
   */
  double getWidthComponent() {
    return kWidthComponent;
  }

  /** Swerve Drive drive mode */
  public enum DriveMode {
    OPEN_LOOP,
    CLOSED_LOOP,
    TELEOP,
    TRAJECTORY,
    AZIMUTH
  }  

  private Rotation2d getGyroAngle(){
  // Get my gyro angle. We are negating the value because gyros return positive
  // values as the robot turns clockwise. This is not standard convention that is
  // used by the WPILib classes.
  return Rotation2d.fromDegrees(Math.IEEEremainder(-gyro.getAngle(), 360));
  }

  public void updateOdometry() {
  // Update the pose
  m_odometry.update(getGyroAngle(), wheels[0].getState(), wheels[1].getState(), wheels[2].getState(), wheels[3].getState()); 
  
  /* Debug Data
  Translation2d xy = m_odometry.getPoseMeters().getTranslation() ;
  SmartDashboard.putNumber("Odometry Angle", gyro.getAngle()); 
  SmartDashboard.putNumber("Distance X", Units.metersToFeet(xy.getX()) * .9); 
  SmartDashboard.putNumber("Distance Y", Units.metersToFeet(xy.getY()) * .9);  
  SmartDashboard.putNumber("Front Right Speed", Units.metersToFeet(wheels[0].getState().speedMetersPerSecond * Constants.VelocityConversions.SensorTimePerSec)); 
  SmartDashboard.putNumber("Front Right Angle", wheels[0].getState().angle.getDegrees()); 
  SmartDashboard.putNumber("Front Left Speed", Units.metersToFeet(wheels[1].getState().speedMetersPerSecond * Constants.VelocityConversions.SensorTimePerSec));
  SmartDashboard.putNumber("Front Left Angle", wheels[1].getState().angle.getDegrees()); 
  SmartDashboard.putNumber("Back Left Speed", Units.metersToFeet(wheels[2].getState().speedMetersPerSecond * Constants.VelocityConversions.SensorTimePerSec));
  SmartDashboard.putNumber("Back Left Angle", wheels[2].getState().angle.getDegrees()); 
  SmartDashboard.putNumber("Back Right Speed", Units.metersToFeet(wheels[3].getState().speedMetersPerSecond * Constants.VelocityConversions.SensorTimePerSec));
  SmartDashboard.putNumber("Back Right Angle", wheels[3].getState().angle.getDegrees()); 
  */


} 

public void setModuleStates(SwerveModuleState[] states){ 
  wheels[0].setState(states[0]);
  wheels[1].setState(states[1]); 
  wheels[2].setState(states[2]);
  wheels[3].setState(states[3]);
}

public void resetOdometry(){ 
  m_odometry.resetPosition(new Pose2d(), getGyroAngle());
}

public void resetOdometry(Pose2d pose){ 
  m_odometry.resetPosition(pose, getGyroAngle());
} 

public Pose2d getPose(){ 
  return m_odometry.getPoseMeters(); 
}
}
