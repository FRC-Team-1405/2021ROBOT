// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  TalonFXSensorCollection collection;
  
  private TalonFX drive;
  private CANCoder rotation;

  private String label;
  public SwerveModule(String label, int rotationId, int driveId){
    rotation = new CANCoder(rotationId);
    drive = new TalonFX(driveId);

    this.label = label;
    collection = drive.getSensorCollection();
    display();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(rotationToMeter(collection.getIntegratedSensorVelocity()), new Rotation2d(degreeToMeter(rotation.getVelocity())));
  }


  //TODO:Add motor ratio to both functions
  private double degreeToMeter(double degree) {
    double meter = degree * 360 * Constants.VelocityConversions.WheelCircumference;
    return meter;
  }

  private double rotationToMeter(double rotation) {
    double meter = rotation * Constants.VelocityConversions.WheelCircumference;
    return meter;
  }

  public void display() {
    SmartDashboard.putNumber(label+"/Rotation Position", rotation.getPosition());
    SmartDashboard.putNumber(label+"/Rotation Absolute Position", rotation.getAbsolutePosition());
    SmartDashboard.putNumber(label+"/Rotation Velocity", rotation.getVelocity());
  
    SmartDashboard.putNumber(label+"/Drive Position", collection.getIntegratedSensorPosition());
    SmartDashboard.putNumber(label+"/Drive Absolute Position", collection.getIntegratedSensorAbsolutePosition());
    SmartDashboard.putNumber(label+"/Drive Velocity", collection.getIntegratedSensorVelocity());
  }
}
