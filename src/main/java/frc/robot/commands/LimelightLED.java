/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.MathTools;
import frc.robot.sensors.LEDStrip;
import frc.robot.sensors.Limelight;

public class LimelightLED extends CommandBase {
  private LEDStrip ledStrip;
  private Limelight limelight;
  private AddressableLEDBuffer addressableLEDBuffer;
  /**
   * Creates a new LimelightLED.
   */
  public LimelightLED(LEDStrip ledStrip, Limelight limelight) {
    this.ledStrip = ledStrip;
    this.limelight = limelight;
    this.addressableLEDBuffer = ledStrip.getLedBuffer() ;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double offset = Math.abs(limelight.getTX());
    int numberOfLeds = MathTools.map((int) offset, 0, 29, 0, Constants.UnderGlow.ledCount);
    if(offset <= Constants.ShooterConstants.limelightError){
      for(int i = 0; i < numberOfLeds; i++){
        if(i%3==0){
          addressableLEDBuffer.setLED(i + Constants.UnderGlow.ledStart, Color.kYellow);
        }else{
          addressableLEDBuffer.setLED(i + Constants.UnderGlow.ledStart, Color.kGreen);
        }
      }
    }else{
      for(int i = 0; i < Constants.UnderGlow.ledCount-numberOfLeds; i++){
        addressableLEDBuffer.setLED(i + Constants.UnderGlow.ledStart, Color.kGreen);
      }
    }


    ledStrip.displayLEDBuffer(addressableLEDBuffer);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
