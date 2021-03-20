/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArcadeDrive;

public class SetStartPos extends CommandBase {
  private ArcadeDrive arcadeDrive;


  public SetStartPos(ArcadeDrive arcadeDrive) {
    SmartDashboard.putNumber("StartingPos/xPos",xPos);
    SmartDashboard.putNumber("StartingPos/yPos",yPos);
    this.arcadeDrive = arcadeDrive;
    addRequirements(this.arcadeDrive);
  }

  private double xPos = 0;
  private double yPos = 0;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xPos = SmartDashboard.getNumber("StartingPos/xPos", xPos);
    yPos = SmartDashboard.getNumber("StartingPos/yPos", yPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
