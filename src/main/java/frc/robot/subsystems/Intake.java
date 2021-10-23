/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */

  private WPI_TalonSRX intakeTalon = new WPI_TalonSRX(Constants.intakeTalon); 
  private WPI_TalonSRX intakeDeploy = new WPI_TalonSRX(Constants.intakeDeploy); 
  

  public Intake() {
    intakeTalon.set(ControlMode.PercentOutput, 0);
    intakeDeploy.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    
  }

  private boolean isDeployed = false;
  public boolean isDeployed(){
    return isDeployed;
  }
  
  public void deploy(){ 
    isDeployed=true;
    intakeDeploy.set(ControlMode.PercentOutput, 0.5);
  } 

  public void retract(){
    if (!deployLocked){
      isDeployed=false;
      intakeDeploy.set(ControlMode.PercentOutput, -0.6);
    }
  } 

  private boolean deployLocked = false;
  public void LockDeployed() {
    deploy();
    deployLocked = true;
  }

  public void enable(){
    intakeTalon.set(ControlMode.PercentOutput, Constants.IntakeConstants.SPEED);
  }

  public void disable(){
    intakeTalon.set(ControlMode.PercentOutput, 0);
  }

  public void outtake(){
    intakeTalon.set(ControlMode.PercentOutput, -Constants.IntakeConstants.SPEED);
  }
}
