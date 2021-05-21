/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.ArcadeDrive;
import frc.robot.subsystems.SwerveDriveBase;

public class TurnToBall extends CommandBase {
  ArcadeDrive arcadeDrive;
  SwerveDriveBase swerveDrive;
  Limelight limelight;
  double turnSpeed; 

  public TurnToBall(ArcadeDrive arcadeDrive, Limelight limelight, double speed) {
    this.arcadeDrive = arcadeDrive;
    this.limelight = limelight;

    addRequirements(arcadeDrive);
  }

  public TurnToBall(SwerveDriveBase swerveDrive, Limelight limelight, double turnSpeed) {
    this.swerveDrive = swerveDrive;
    this.turnSpeed = turnSpeed; 
    this.limelight = limelight;

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    limelight.setPipeline((byte) 9);
    limelight.setLED(Limelight.LED.Default);
  }

  @Override
  public void execute() {
    if (arcadeDrive != null){
      arcadeDrive.driveRobot(0, turnSpeed, false);
    }
    if (swerveDrive != null) {
      swerveDrive.driveStraight(0, turnSpeed, 1.0);
    }
  }

  @Override
  public boolean isFinished() {
    if(limelight.hasTarget() && Math.abs(limelight.getTX()) <= 20.0)
    {
      if (arcadeDrive != null){
        arcadeDrive.stop();
      }
      if (swerveDrive != null){
        swerveDrive.stop();
      }
      return true;
    }
    return false;
  }
}
