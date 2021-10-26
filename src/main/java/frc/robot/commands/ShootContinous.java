// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class ShootContinous extends CommandBase { 
  private Shooter shooter; 
  private Hood hood; 
  private DoubleSupplier speed; 
  private DoubleSupplier angle; 
  private Limelight limelight;
  private boolean isPrepared; 
  private boolean recalculate; 
  private BooleanSupplier isOveridden; 
  
  /** Creates a new PrepareShooter. */ 
  public ShootContinous(Shooter shooter, Hood hood, DoubleSupplier speed, DoubleSupplier angle, Limelight limelight, boolean recalculate, BooleanSupplier isOveridden) {
    // Use addRequirements() here to declare subsystem dependencies. 
    this.shooter = shooter; 
    this.hood = hood; 
    this.speed = speed; 
    this.angle = angle; 
    this.limelight = limelight;
    this.recalculate = recalculate; 
    this.isOveridden = isOveridden; 

    this.addRequirements(shooter, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setVelocity((int) speed.getAsDouble()); 
    hood.setPosition((int) angle.getAsDouble());
    limelight.setPipeline(Constants.LimelightConfig.TargetPipeline);
    limelight.setLED(Limelight.LED.On);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    if (this.recalculate){
      shooter.setVelocity((int) speed.getAsDouble()); 
      hood.setPosition((int) angle.getAsDouble());  
    }

    boolean shooterReady = shooter.isAtVelocity();
    boolean hoodReady = hood.isReady();

    SmartDashboard.putBoolean("shooter VelocitySet", shooterReady);
    SmartDashboard.putBoolean("shooter HoodSet", hoodReady);

    isPrepared = shooterReady && hoodReady; 

    if(isPrepared || isOveridden.getAsBoolean()){ 
      shooter.index();
    }else{ 
      shooter.close();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    shooter.close();

    limelight.setLED(Limelight.LED.Off);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
