// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.DistanceToPower;
import frc.robot.lib.Interpolate;
import frc.robot.sensors.LidarLitePWM;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDriveBase;

public class ShootConstantly extends CommandBase {
  private Shooter shooter; 
  private Hood hood; 
  private SwerveDriveBase driveBase; 
  public Limelight camera = new Limelight();  
  //private LidarLitePWM aimingLidar;

  private Interpolate distanceToAngle = new Interpolate("ToDo");

  public ShootConstantly(Shooter shooter, Hood hood) {
    this.shooter = shooter; 
    this.hood = hood;
    //this.aimingLidar = aimingLidar;
    addRequirements(shooter);
    addRequirements(hood);
  }
  
  @Override
  public void initialize() {    
  } 
  
  @Override
  public void execute(){ 
    double distance = 0; //aimingLidar.getDistance();
    double velocity = DistanceToPower.calculatePower(distance);
    double angle = distanceToAngle.CalculateOutput(distance);
    SmartDashboard.putNumber("Shooter/Distance", distance);
    SmartDashboard.putNumber("Shooter/Velocity", velocity);
    SmartDashboard.putNumber("Shooter/Angle", angle);

    hood.setPosition((int)angle);
    // shooter.setVelocity((int)velocity);

    if (hood.isReady() && shooter.isAtVelocity()){
      // fire the trigger
    }
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    shooter.stop();
    hood.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
