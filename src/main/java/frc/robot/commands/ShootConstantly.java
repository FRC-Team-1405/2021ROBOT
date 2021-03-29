// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootConstantly extends CommandBase {
  Shooter shooter;
  int velocity; 

  public ShootConstantly(Shooter shooter, int velocity) {
    this.shooter = shooter;
    this.velocity = velocity;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setVelocity(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
