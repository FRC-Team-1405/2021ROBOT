// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private TalonFX drive;
  private CANCoder rotation;

  private String label;
  public SwerveModule(String label, int rotationId, int driveId){
    rotation = new CANCoder(rotationId);
    drive = new TalonFX(driveId);

    this.label = label;
    display();
  }

  public void display() {
    SmartDashboard.putNumber(label+"/Rotation Position", rotation.getPosition());
    SmartDashboard.putNumber(label+"/Rotation Absolute Position", rotation.getAbsolutePosition());
    SmartDashboard.putNumber(label+"/Rotation Velocity", rotation.getVelocity());
  
    TalonFXSensorCollection collection = drive.getSensorCollection();
    SmartDashboard.putNumber(label+"/Drive Position", collection.getIntegratedSensorPosition());
    SmartDashboard.putNumber(label+"/Drive Absolute Position", collection.getIntegratedSensorAbsolutePosition());
    SmartDashboard.putNumber(label+"/Drive Velocity", collection.getIntegratedSensorVelocity());
  }
}
