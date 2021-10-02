// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Swerve extends SubsystemBase {
    private AHRS gyro;
    private SwerveDriveKinematics m_kinematics;
    
    public Swerve(AHRS gyro) {
    this.gyro = gyro;

    // Creating my kinematics object using the module locations
    m_kinematics = new SwerveDriveKinematics(
      Constants.SwerveBase.backRightLocation, 
      Constants.SwerveBase.frontRightLocation, 
      Constants.SwerveBase.backLeftLocation, 
      Constants.SwerveBase.backRightLocation
    );
    
    SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, getGyroAngle());
  }

  private Rotation2d getGyroAngle(){
    // Get my gyro angle. We are negating the value because gyros return positive
    // values as the robot turns clockwise. This is not standard convention that is
    // used by the WPILib classes.
      return Rotation2d.fromDegrees(Math.IEEEremainder(-gyro.getAngle(), 360));
  }

  //https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java#L41
  public void drive(double xSpeed, double ySpeed, double rot){
    SwerveModuleState[] swerveModuleStates = 
        m_kinematics.toSwerveModuleStates(
          new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.SwerveBase.maxSpeed);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
