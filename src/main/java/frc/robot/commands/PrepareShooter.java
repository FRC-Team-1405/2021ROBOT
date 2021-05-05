// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class PrepareShooter extends CommandBase { 
  public Shooter shooter; 
  public Hood hood; 
  public DoubleSupplier speed; 
  public DoubleSupplier angle; 
  public DoubleSupplier distance; 
  public boolean isPrepared; 


  /** Creates a new PrepareShooter. */ 
  public PrepareShooter(Shooter shooter, Hood hood, DoubleSupplier speed, DoubleSupplier angle, DoubleSupplier distance) {
    // Use addRequirements() here to declare subsystem dependencies. 
    this.shooter = shooter; 
    this.hood = hood; 
    this.speed = speed; 
    this.angle = angle; 
    this.distance = distance; 

    this.addRequirements(shooter, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setVelocity((int) speed.getAsDouble()); 
    hood.setPosition((int) angle.getAsDouble());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    isPrepared = shooter.isAtVelocity() && hood.isReady(); 

    if(isPrepared = true){ 
      shooter.index();
    }else{ 
      shooter.close();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isPrepared;
  }
}
