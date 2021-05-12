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

  private WPI_TalonSRX angle = new WPI_TalonSRX(Constants.shooterAngle);
  private static final double ZEROIZE_SPEED = -0.4;

  public Hood() {
    stop();
  }

  @Override
  public void periodic() {
    if (zeroizeActive) {
      if (angle.isRevLimitSwitchClosed() == 1){
        angle.set(ControlMode.PercentOutput, 0);
        zeroizeActive = false;
        zeroizeComplete = true;
      }
    }
  }

  private boolean zeroizeActive = false;
  private boolean zeroizeComplete = false;
  public void zeroize(){
    if (!zeroizeActive){
      zeroizeActive = true;
      angle.set(ControlMode.PercentOutput, ZEROIZE_SPEED);
    }
  }

  public boolean zeroizeComplete(){
    return zeroizeComplete;
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
    int position = angle.getSelectedSensorPosition(); 
    return position;
  };

  public boolean isReady(){
    return Math.abs(getPosition() - targetAngle) < Constants.Hood.maxError;
  }
}
