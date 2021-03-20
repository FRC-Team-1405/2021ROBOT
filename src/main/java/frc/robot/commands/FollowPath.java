/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.ArcadeDrive;
import frc.robot.Constants;

public class FollowPath extends RamseteCommand {
  /**
   * Creates a new FollowPath.
   */
  public FollowPath(Trajectory trajectory, ArcadeDrive driveSubsystem) {
    super(trajectory, 
          driveSubsystem::getPose, 
          new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), 
          Constants.kDriveKinematics, 
          driveSubsystem::setVelocity, 
          driveSubsystem); 
         
  }
}
