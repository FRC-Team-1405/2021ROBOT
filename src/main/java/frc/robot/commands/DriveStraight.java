/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArcadeDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveStraight extends PIDCommand {
  /**
   * Creates a new DriveStraight.
   */
  public DriveStraight(ArcadeDrive driveBase, DoubleSupplier speedSupplier) {
    super(
        // The controller that the command will use
        new PIDController(Constants.DriveStraightPID.kP, Constants.DriveStraightPID.kI, Constants.DriveStraightPID.kD),
        // This should return the measurement
        driveBase::getRate,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> driveBase.driveRobot(speedSupplier, output, false)
        );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(driveBase);
    SmartDashboard.putNumber("DriveStraightPID/kP", Constants.TurnPID.kP);
    SmartDashboard.putNumber("DriveStraightPID/kI", Constants.TurnPID.kI);
    SmartDashboard.putNumber("DriveStraightPID/kD", Constants.TurnPID.kD);
    this.getController().setTolerance(0.5);
  }

  public DriveStraight(ArcadeDrive driveBase, Double speed) {
    super(
        // The controller that the command will use
        new PIDController(Constants.DriveStraightPID.kP, Constants.DriveStraightPID.kI, Constants.DriveStraightPID.kD),
        // This should return the measurement
        driveBase::getRate,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> driveBase.driveRobot(speed, output, false)
        );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(driveBase);
    SmartDashboard.putNumber("DriveStraightPID/kP",Constants.TurnPID.kP);
    SmartDashboard.putNumber("DriveStraightPID/kI",Constants.TurnPID.kI);
    SmartDashboard.putNumber("DriveStraightPID/kD",Constants.TurnPID.kD);
    this.getController().setTolerance(0.5);
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
