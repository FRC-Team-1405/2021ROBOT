/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.IntSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase; 
import frc.robot.subsystems.Shooter;

public class TestShooter extends CommandBase {
  /**
   * Creates a new TestShoot.
   */
  private Shooter shooter;
  private IntSupplier pov;
  private int lastPovValue = -1;
  public TestShooter(Shooter shooter, IntSupplier pov) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.pov = pov;
    addRequirements(this.shooter); 
    SmartDashboard.putNumber("TestShooter/Low", 0); 
    SmartDashboard.putNumber("TestShooter/High", 0);
    SmartDashboard.putNumber("TestShooter/Spin", 0); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("TestShooter/High", 0); 
    SmartDashboard.putNumber("TestShooter/Low", 0); 
    SmartDashboard.putNumber("TestShooter/Spin", 0); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    double high = SmartDashboard.getNumber("TestShooter/High", 0); 
    double low  = SmartDashboard.getNumber("TestShooter/Low", 0); 
    double spin = SmartDashboard.getNumber("TestShooter/Spin", 0) / 2.0; 
    double power = (high + low) / 2.0; 
    SmartDashboard.putNumber("Power", power); 

    if(spin < 0) 
      shooter.launch(-power + (power * Math.abs(spin)), power - (power * Math.abs(spin))); 
    else if(spin>0)
      shooter.launch(-power - (power * Math.abs(spin)), power + (power * Math.abs(spin))); 
    else
      shooter.launch(-power, power); 

    int currentPov = pov.getAsInt();
    if (currentPov == -1)
    {
      switch(lastPovValue){
        case 0: 
          SmartDashboard.putNumber("TestShooter/Low", power); 
          lastPovValue = currentPov; 
           break;
        case 180: 
          SmartDashboard.putNumber("TestShooter/High", power); 
          lastPovValue = currentPov; 
          break;
      }
    } else if (currentPov == 0 || currentPov == 180) {
      lastPovValue = currentPov;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.launch(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
