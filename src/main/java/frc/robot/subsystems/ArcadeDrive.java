/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ArcadeDrive extends SubsystemBase {
  /**
   * Creates a new ArcadeDrive.
   */
  
  WPI_TalonFX driveLeft = new WPI_TalonFX(Constants.driveLeft);
  WPI_TalonFX driveRight = new WPI_TalonFX(Constants.driveRight);
  WPI_TalonFX driveLeftSlave = new WPI_TalonFX(Constants.driveLeftSlave);
  WPI_TalonFX driveRightSlave = new WPI_TalonFX(Constants.driveRightSlave);
  DifferentialDrive driveBase = new DifferentialDrive(driveLeft, driveRight); 

  public DigitalInput driveBaseIndicator = new DigitalInput(1); 

  private AHRS gyro = new AHRS(I2C.Port.kMXP);
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  boolean driveForward = true;
  
  public ArcadeDrive() {
    SmartDashboard.putBoolean("Drive Forward", driveForward);
    driveLeft.configNeutralDeadband(0.04, 10);
    driveRight.configNeutralDeadband(0.04, 10);

    driveLeft.set(ControlMode.PercentOutput, 0);
    driveRight.set(ControlMode.PercentOutput, 0);
    driveLeftSlave.set(ControlMode.Follower, Constants.driveLeft);
    driveRightSlave.set(ControlMode.Follower, Constants.driveRight);

    driveBase.setDeadband(0.0);

    driveLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    driveRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  public void toggleDriveDirection(){
    driveForward = !driveForward;
    SmartDashboard.putBoolean("Drive Forward", driveForward);
  }

  public void driveRobot(double xSpeed, double zRotation, boolean squareInputs){
    if(driveForward){
      driveBase.arcadeDrive(xSpeed, zRotation, squareInputs);
    }else{
      driveBase.arcadeDrive(-xSpeed, zRotation, squareInputs);
    }
  }

  public void driveRobot(DoubleSupplier xSpeedSupplier, double zRotation, boolean squareInputs){
    if(driveForward){
      driveBase.arcadeDrive(xSpeedSupplier.getAsDouble(), zRotation, squareInputs);
    }else{
      driveBase.arcadeDrive(-xSpeedSupplier.getAsDouble(), zRotation, squareInputs);
    }
  }

  public void resetDistance(){
    driveLeft.setSelectedSensorPosition(0);
    driveRight.setSelectedSensorPosition(0);
  }

  public void stop(){
    driveLeft.set(ControlMode.PercentOutput, 0);
    driveRight.set(ControlMode.PercentOutput, 0);
  }

  public double getVelocity(){
   return Math.sqrt(gyro.getVelocityX()*gyro.getVelocityX() + gyro.getVelocityY()*gyro.getVelocityY());
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360.0);
  }

  public double getRoll(){
    return gyro.getRoll();
  }

  public double getRate(){
    return gyro.getRate();
  }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run 

      
      if(!Robot.fmsAttached){
        SmartDashboard.putNumber("Velocity", getVelocity());
        SmartDashboard.putNumber("Heading", getHeading());
      }
      odometry.update(Rotation2d.fromDegrees(-getHeading()), driveLeft.getSelectedSensorPosition()*Constants.VelocityConversions.SensorVelocityToMetersPerSecond,
                      -driveRight.getSelectedSensorPosition()*Constants.VelocityConversions.SensorVelocityToMetersPerSecond);
    }
    public void resetPosition(){
      resetPosition(0,0);
    }
    public void resetPosition(double xPos, double yPos){
      gyro.reset();;
      resetOdometry( new Pose2d(xPos,yPos, Rotation2d.fromDegrees(0.0)) );
    }

    public Pose2d getPose() {
      return odometry.getPoseMeters();
    }
    public void resetOdometry(Pose2d pose) {
      driveLeft.setSelectedSensorPosition(0);
      driveRight.setSelectedSensorPosition(0);
      odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }
     
    public void setVelocity(double leftSpeed, double rightSpeed){
      if(!Robot.fmsAttached){
        SmartDashboard.putNumber("ArcadeDrive/Speed Left", Constants.VelocityConversions.MetersPerSecondToVelocity*leftSpeed);
        SmartDashboard.putNumber("ArcadeDrive/Speed Right", Constants.VelocityConversions.MetersPerSecondToVelocity*rightSpeed);
      }
        driveLeft.set(ControlMode.Velocity, Constants.VelocityConversions.MetersPerSecondToVelocity*leftSpeed); 
      driveRight.set(ControlMode.Velocity, - Constants.VelocityConversions.MetersPerSecondToVelocity*rightSpeed); 
    }
    public void resetEncoder(){
      driveLeft.setSelectedSensorPosition(0);
      driveRight.setSelectedSensorPosition(0);
      if(!Robot.fmsAttached){
        SmartDashboard.putNumber("ArcadeDrive/Distance", 0);
      }
    }
    public double getDistance(){
      double distance = (((driveLeft.getSelectedSensorPosition()
              -driveRight.getSelectedSensorPosition())/2.0)
                  *Constants.VelocityConversions.SensorVelocityToMetersPerSecond);
      if(!Robot.fmsAttached){
        SmartDashboard.putNumber("ArcadeDrive/Distance", distance);
      }
      return distance;

    }
}
