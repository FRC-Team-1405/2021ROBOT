/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveIntake extends SubsystemBase {
  /**
   * Creates a new SwerveIntake.
   */ 

  public WPI_TalonSRX intakeDeployTalon = new WPI_TalonSRX(Constants.swerveIntakeDeployTalon); 
  public WPI_TalonSRX intakeTalon = new WPI_TalonSRX(Constants.swerveIntakeTalon); 
  public boolean isDeployed = false; 

  public SwerveIntake() { 

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } 

  public void intake(){ 
    intakeTalon.set(1.0);
  } 

  public void stop(){ 
    intakeTalon.set(0);
  }

  public void deploy(){ 
    isDeployed = true; 
  } 

  public void retract(){ 
    isDeployed = false; 
  } 
}
