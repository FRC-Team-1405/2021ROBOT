// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.Limelight.LED;
import frc.robot.subsystems.SwerveDriveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveByAngle extends PIDCommand {

  static Preferences prefs = Preferences.getInstance();
  static {
    prefs.initDouble("DriveByAngle/P",0.01);
    prefs.initDouble("DriveByAngle/I",0.0001);
    prefs.initDouble("DriveByAngle/D",0);
  }
  /** Creates a new DriveByAngle. */
  public DriveByAngle(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier speedLimit, DoubleSupplier sensor, SwerveDriveBase drive) {
    super(
        // The controller that the command will use
        new PIDController(prefs.getDouble("DriveByAngle/P",0.01),
                          prefs.getDouble("DriveByAngle/I",0.0001),
                          prefs.getDouble("DriveByAngle/D",0)),
        // This should return the measurement
        sensor,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          SmartDashboard.putNumber("DriveByAngle output", output);
          drive.drive(forward.getAsDouble(), strafe.getAsDouble(), output, speedLimit.getAsDouble());
        });

      this.sensor = sensor;
      this.m_controller.enableContinuousInput(-180.0, 180.0);
      this.m_controller.setTolerance(1.0
      );
      SmartDashboard.putData("DriveByAngle PID", m_controller);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  protected DoubleSupplier sensor;

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("DriveByAngle Target", m_controller.getSetpoint());
    SmartDashboard.putNumber("DriveByAngle error", m_controller.getPositionError());
    SmartDashboard.putNumber("DriveByAngle sensor", sensor.getAsDouble());
    super.execute();
  }
}
