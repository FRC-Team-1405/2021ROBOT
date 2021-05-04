// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.IntSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  private int targetAngle;

  WPI_TalonSRX angle = new WPI_TalonSRX(Constants.shooterAngle);

  public Hood() {
    stop();
  }

  public void stop(){
    angle.set(ControlMode.PercentOutput, 0);
    targetAngle = getPosition();
  }

  public void setPosition(int position) {
    targetAngle = position;
    angle.set(ControlMode.MotionMagic, targetAngle);
  };

  public int getPosition() {
    int position = angle.getSensorCollection().getPulseWidthPosition(); 
    return position;
  };

  public boolean isReady(){
    return Math.abs(getPosition() - targetAngle) < Constants.Hood.maxError;
  }
}
