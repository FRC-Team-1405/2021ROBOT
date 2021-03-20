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
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Balance extends PIDCommand {
  /**
   * Creates a new Balance.
   */
  public Balance(Climber climber, DoubleSupplier angle) {
    super(
        // The controller that the command will use
        new PIDController(Constants.BalancePID.kP, Constants.BalancePID.kI, Constants.BalancePID.kD),
        // This should return the measurement
        angle,
        // This should return the setpoint (can also be a constant)
        Math.atan( (climber.getLeftScissorLength() - climber.getRightScissorLength() - Constants.robotWidth*Math.sin(angle.getAsDouble())) / (Constants.robotWidth*Math.cos(angle.getAsDouble())) ),
        // This uses the output
        output -> {climber.moveLeft(output); climber.moveRight(-output);}
        );
    addRequirements(climber);
    SmartDashboard.putNumber("BalancePID/kP",Constants.BalancePID.kP);
    SmartDashboard.putNumber("BalancePID/kI",Constants.BalancePID.kI);
    SmartDashboard.putNumber("BalancePID/kD",Constants.BalancePID.kD);
    this.getController().setTolerance(0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
