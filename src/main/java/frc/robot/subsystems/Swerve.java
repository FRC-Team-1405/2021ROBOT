// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Swerve extends SubsystemBase {
  public Swerve() {

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      Constants.SwerveBase.backRightLocation, 
      Constants.SwerveBase.frontRightLocation, 
      Constants.SwerveBase.backLeftLocation, 
      Constants.SwerveBase.backRightLocation
    );
    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
