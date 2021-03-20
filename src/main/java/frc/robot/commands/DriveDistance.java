/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ArcadeDrive;

public class DriveDistance extends DriveStraight {
  /**
   * Creates a new DriveDistance.
   */
  private ArcadeDrive driveBase;
  private double distance;
  public DriveDistance(ArcadeDrive driveBase, double distance, double speed) {
    super(driveBase, speed);
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveBase = driveBase;
    this.distance = distance;
    addRequirements(this.driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveBase.resetDistance();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(driveBase.getDistance()) >= Math.abs(distance);
  }
}
