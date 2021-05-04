// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.LidarLitePWM;

public class Shooter extends SubsystemBase {
  TalonSRX master = new TalonSRX(Constants.shooterMaster);
  
  WPI_TalonFX slave = new WPI_TalonFX(Constants.shooterSlave); 
  
  

  Servo trigger = new Servo(0); 
  
  SlewRateLimiter rateLimit = new SlewRateLimiter(2500, 0); 

  int targetVelocity = 0;

  public Shooter(){    
    slave.setInverted(true);
    slave.follow(master);  

    SmartDashboard.putNumber("Servo Test Value", 0); 
    SmartDashboard.putNumber("Shooter Test Value", 0);

    close();
    
}

  @Override
  public void periodic() {
    int newVelocity = (int)rateLimit.calculate(targetVelocity);
    if(targetVelocity != newVelocity)
      master.set(ControlMode.Velocity, newVelocity); 
    //trigger.set(SmartDashboard.getNumber("Servo Test Value", 0)); 
    //Aiming lidar returns in cms so we make it feet cuz this isn't Europe or really anywhere but the US :shrug:
    //SmartDashboard.putNumber("Lidar Distance", Units.metersToFeet(aimingLidar.getDistance() / 100));
  }

  public void setVelocity(int velocity){
    targetVelocity = velocity;
    master.set(ControlMode.Velocity, rateLimit.calculate(velocity));
  }

  public boolean isAtVelocity(){
    return master.getClosedLoopError() <= Constants.ShooterConstants.acceptableError;
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

  public void testShoot(){ 
    master.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Shooter Test Value", 0)); 
  }
}
