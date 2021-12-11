// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.thirdcoast.swerve.SwerveDrive.DriveMode;
import frc.robot.subsystems.SwerveDriveBase;

public class DriveRobotCentric extends CommandBase {
  /** Creates a new DriveRobotCentric. */  

  private static final double DEADBAND = 0.2;

  private SwerveDriveBase driveBase;
  private DoubleSupplier getForward;
  private DoubleSupplier getStrafe; 
  private DoubleSupplier speedLimit; 
  private DoubleSupplier getYaw;

public DriveRobotCentric(DoubleSupplier getForward, DoubleSupplier getStrafe, DoubleSupplier getYaw, DoubleSupplier speedLimit, SwerveDriveBase driveBase) {
    // Use addRequirements() here to declare subsystem dependencies. 
    this.getForward = getForward;
    this.getStrafe = getStrafe;
    this.getYaw = getYaw; 
    this.speedLimit = speedLimit; 
    this.driveBase = driveBase;
    addRequirements(driveBase);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    driveBase.setDriveMode(DriveMode.TELEOP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = deadband(getForward.getAsDouble());
    double strafe = deadband(getStrafe.getAsDouble());
    double azimuth = deadbandAzimuth(getYaw.getAsDouble())/2.0; 

    driveBase.driveRobotCentric(forward, strafe, azimuth, speedLimit.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    driveBase.drive(0.0, 0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  } 

  private double deadband(double value) {
    if (Math.abs(value) < DEADBAND) return 0.0;
    return value;
  } 

  private double deadbandAzimuth(double value) {
    if (Math.abs(value) < DEADBAND*2) return 0.0;
    return value;
  }
}
