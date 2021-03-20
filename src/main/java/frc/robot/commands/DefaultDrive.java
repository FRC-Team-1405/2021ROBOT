/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArcadeDrive;

public class DefaultDrive extends CommandBase {
  private DoubleSupplier driveSpeed;
  private DoubleSupplier driveRotation;
  private ArcadeDrive driveBase;
  /**
   * Creates a new DefaultDrive.
   */
  public DefaultDrive(DoubleSupplier driveSpeed, DoubleSupplier driveRotation, ArcadeDrive driveBase) {
    this.driveSpeed = driveSpeed;
    this.driveRotation = driveRotation;
    this.driveBase = driveBase;
    addRequirements(this.driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveBase.resetDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveBase.driveRobot( driveSpeed.getAsDouble(), driveRotation.getAsDouble(), true);
    driveBase.getDistance();
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
