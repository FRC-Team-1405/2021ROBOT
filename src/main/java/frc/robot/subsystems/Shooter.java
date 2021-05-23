// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  TalonSRX master = new TalonSRX(Constants.shooterMaster);
  TalonSRX slave = new TalonSRX(Constants.shooterSlave); 

  Servo trigger = new Servo(0); 
  
  SlewRateLimiter rateLimit = new SlewRateLimiter(10000, 0); 

  int targetVelocity = 0;

  public Shooter(){    
    slave.setInverted(true);
    slave.follow(master);  
    close();
  }

  @Override
  public void periodic() {
    int newVelocity = (int)rateLimit.calculate(targetVelocity);
    if((int)master.getClosedLoopTarget() != newVelocity)
      master.set(ControlMode.Velocity, newVelocity); 
  }

  public void setVelocity(int velocity){
    targetVelocity = velocity;
    master.set(ControlMode.Velocity, rateLimit.calculate(velocity));
  }

  public int getVelocity(){
    return (int)master.getClosedLoopTarget() - master.getClosedLoopError();
  }
  public boolean isAtVelocity(){
    return master.getClosedLoopTarget() == targetVelocity && Math.abs(master.getClosedLoopError()) <= Constants.ShooterConstants.acceptableError;
  }

  public void stop(){
    targetVelocity = 0;
    rateLimit.reset(0);
    master.set(ControlMode.PercentOutput, 0);
  } 

  public void index(){ 
    trigger.set(0.2); 
  } 

  public void close(){ 
    trigger.set(0.4); 
  } 
}
