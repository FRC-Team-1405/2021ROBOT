/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveBase;
import frc.robot.Constants;
import frc.robot.lib.SmartSupplier;
import frc.robot.lib.thirdcoast.swerve.SwerveDrive.DriveMode;


public final class SwerveDrive extends CommandBase {
  private static final Logger logger = Logger.getLogger(SwerveDrive.class.getName());

  private static final double DEADBAND = 0.2;

  private SwerveDriveBase driveBase;
  private DoubleSupplier getForward;
  private DoubleSupplier getStrafe; 
  private DoubleSupplier speedLimit; 
  private DoubleSupplier getYaw;
  private DoubleSupplier getHeading;
  private double headingTarget;

  private PIDController thetaController =
  new PIDController(
      new SmartSupplier("Swerve Teleop rP", 0.1).getAsDouble(),
      new SmartSupplier("Swerve Teleop rI", 0).getAsDouble(), 
      new SmartSupplier("Swerve Teleop rD", 0).getAsDouble());

  // private ProfiledPIDController thetaController =
  // new ProfiledPIDController(
  //     new SmartSupplier("Swerve Teleop rP", 0.75).getAsDouble(),
  //     new SmartSupplier("Swerve Teleop rI", 0).getAsDouble(), 
  //     new SmartSupplier("Swerve Teleop rD", 0).getAsDouble(), 
  //     new TrapezoidProfile.Constraints(Constants.SwerveBase.maxAngularSpeed, Constants.SwerveBase.maxAngularAccelerartion));

  public SwerveDrive(DoubleSupplier getForward, DoubleSupplier getStrafe, DoubleSupplier getYaw, DoubleSupplier getHeading, DoubleSupplier speedLimit, SwerveDriveBase driveBase) {
    this.getForward = getForward;
    this.getStrafe = getStrafe;
    this.getYaw = getYaw; 
    this.getHeading = getHeading;
    this.speedLimit = speedLimit; 
    this.driveBase = driveBase;
    addRequirements(driveBase);
  }

  @Override
  public void initialize() {
    driveBase.setDriveMode(DriveMode.TELEOP); 
    thetaController.enableContinuousInput(-Math.PI, Math.PI); 
    headingTarget = getHeading.getAsDouble();
    thetaController.setSetpoint(headingTarget);
    thetaController.setTolerance(5.0);
    // thetaController.setGoal( headingTarget ) ;
  }

  @Override
  public void execute() {
    double forward = deadband(getForward.getAsDouble());
    double strafe = deadband(getStrafe.getAsDouble());
    double azimuth = deadbandAzimuth(getYaw.getAsDouble())/2.0;
    // if (azimuth != 0.0){
    //   headingTarget = getHeading.getAsDouble() + azimuth * 10.0; 
    // }
    logger.fine( () -> String.format("Forward: %f Strafe: %f Azimuth %f", forward, strafe, azimuth)) ;

    double azimuthValue = thetaController.calculate(getHeading.getAsDouble(), headingTarget);

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

  private double deadbandAzimuth(double value) {
    if (Math.abs(value) < DEADBAND*2) return 0.0;
    return value;
  }

}
