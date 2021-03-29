// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */  
  private int zeroAngle; 
  private int targetAngle;

  WPI_TalonSRX angle = new WPI_TalonSRX(Constants.shooterAngle); 
  private static final String PREF_KEY = "Hood Zero";

  public Hood() { 
    zeroAngle = Preferences.getInstance().getInt(PREF_KEY, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 
  } 

  public void setPosition(int position) {
    targetAngle = position + zeroAngle;
    angle.set(ControlMode.MotionMagic, targetAngle);
  };

  public int getPosition() {
    int position = angle.getSensorCollection().getPulseWidthPosition(); 
    return position - zeroAngle;
  };

  public boolean isReady(){
    return Math.abs(getPosition() - targetAngle) < Constants.Hood.maxError;
  }

  public void zeroize(){ 
    zeroAngle = angle.getSensorCollection().getPulseWidthPosition(); 
    Preferences.getInstance().putInt(PREF_KEY, zeroAngle);
  }
}
