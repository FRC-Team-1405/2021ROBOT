/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.sensors.LEDStrip;

public class UnderGlow extends CommandBase {
  private LEDStrip ledStrip;
  private AddressableLEDBuffer addressableLEDBuffer;
  private Alliance team = Alliance.Invalid;
  private Color teamColor = Color.kForestGreen;


  public UnderGlow(LEDStrip ledStrip) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ledStrip = ledStrip;
    this.addressableLEDBuffer = ledStrip.getLedBuffer() ;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    team = DriverStation.getInstance().getAlliance();

    if(team == Alliance.Blue)
    {
      teamColor = Color.kBlue;
    }
    else if(team == Alliance.Red)
    {
      teamColor = Color.kRed;
    }
    else{
      teamColor = Color.kForestGreen;
    }

    for(int i = 0; i < Constants.UnderGlow.ledCount; i++)
    {
      addressableLEDBuffer.setLED(i + Constants.UnderGlow.ledStart, teamColor);
    }

    ledStrip.displayLEDBuffer(addressableLEDBuffer);
  }

  public void resetTeamColor(){
    team = Alliance.Invalid;
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  (team != Alliance.Invalid);
  }
}
