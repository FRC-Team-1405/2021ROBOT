// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 
import frc.robot.Constants;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  private int targetAngle;

  private WPI_TalonSRX angle = new WPI_TalonSRX(Constants.shooterAngle);
  private static final double ZEROIZE_SPEED = -0.4; 

  private enum Zeroize {
    Initialize{
      public Zeroize execute(TalonSRX motor) {
        motor.set(ControlMode.PercentOutput, ZEROIZE_SPEED);
        return Zeroize;
      };
    },
    Zeroize{
      public Zeroize execute(TalonSRX motor) {
        if (motor.isRevLimitSwitchClosed() == 1){
          motor.set(ControlMode.PercentOutput, 0);
          return Ready;
        }
        return this;
      };
    },
    Ready{
      public Zeroize execute(TalonSRX motor) {
        return this;
      };
    };
    public abstract Zeroize execute(TalonSRX motor);
  };
  
  
  private Zeroize zeroizeState = Zeroize.Initialize;

  public Hood() {
    stop(); 
  }

  @Override
  public void periodic() {
    zeroizeState = zeroizeState.execute(angle); 
  }

  public void zeroize(){
    zeroizeState = Zeroize.Initialize;
  }

  public boolean zeroizeComplete(){
    return zeroizeState == Zeroize.Ready;
  }

  public void stop(){
    angle.set(ControlMode.PercentOutput, 0);
    targetAngle = getPosition();
  }

  public void setPosition(int position) {
    if (zeroizeState != Zeroize.Ready)
      return ;

    if (Math.abs(targetAngle - position) > 250){
      System.out.printf("Hood Position: %d\n", position);
    }
    targetAngle = position;
    angle.set(ControlMode.MotionMagic, targetAngle);
  };

  public int getPosition() {
    if (zeroizeState != Zeroize.Ready)
      return 0;

      int position = (int) angle.getSelectedSensorPosition(); 
    return position;
  };

  public boolean isReady(){
    return Math.abs(getPosition() - targetAngle) < Constants.Hood.maxError;
  } 

  }
