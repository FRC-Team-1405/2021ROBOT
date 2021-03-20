/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.logging.Logger;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveBase;
import frc.robot.lib.thirdcoast.swerve.SwerveDrive.DriveMode;


public final class SwerveDrive extends CommandBase {
  private static final Logger logger = Logger.getLogger(SwerveDrive.class.getName());

  private static final double DEADBAND = 0.2;

  private SwerveDriveBase driveBase;
  private DoubleSupplier getForward;
  private DoubleSupplier getStrafe; 
  private DoubleSupplier speedLimit; 
  private DoubleSupplier getYaw;

  public SwerveDrive(DoubleSupplier getForward, DoubleSupplier getStrafe, DoubleSupplier getYaw, DoubleSupplier speedLimit, SwerveDriveBase driveBase) {
    this.getForward = getForward;
    this.getStrafe = getStrafe;
    this.getYaw = getYaw; 
    this.speedLimit = speedLimit; 
    this.driveBase = driveBase;
    addRequirements(driveBase);
  }

  @Override
  public void initialize() {
    driveBase.setDriveMode(DriveMode.TELEOP);
  }

  @Override
  public void execute() {
    double forward = deadband(getForward.getAsDouble());
    double strafe = deadband(getStrafe.getAsDouble());
    double azimuth = deadband(getYaw.getAsDouble()); 
    logger.fine( () -> String.format("Forward: %f Strafe: %f Azimuth %f", forward, strafe, azimuth)) ;
    driveBase.drive(forward, strafe, azimuth, speedLimit.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.drive(0.0, 0.0, 0.0, 0.0);
  }

  private double deadband(double value) {
    if (Math.abs(value) < DEADBAND) return 0.0;
    return value;
  }
}
