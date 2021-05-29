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

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */

  private WPI_TalonSRX intakeTalon = new WPI_TalonSRX(Constants.intakeTalon); 
  private Solenoid intakeDeploy = new Solenoid(Constants.intakeDeploy); 

  public Intake() {
    intakeTalon.set(ControlMode.PercentOutput, 0);
    intakeDeploy.set(false);
  }

  @Override
  public void periodic() {
    
  }

  public void deploy(){ 
    intakeDeploy.set(true);
  } 

  public void retract(){
    intakeDeploy.set(false);
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
