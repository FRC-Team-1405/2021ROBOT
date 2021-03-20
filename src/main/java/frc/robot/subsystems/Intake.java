/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */

  private WPI_TalonSRX intakeTalon = new WPI_TalonSRX(Constants.intakeTalon);
  private WPI_TalonSRX intakeDeploy = new WPI_TalonSRX(Constants.intakeDeploy);
  private int errorThreshold = 50;
  private int loopsToSettle = 10;
  private int withinThresholdLoops = 0;
  private int targetPosition = Constants.IntakeConstants.RETRACT_POSITION;
  private boolean movingArm = false;

  public Intake() {
    intakeTalon.set(ControlMode.PercentOutput, 0);
    intakeDeploy.set(0);

    intakeDeploy.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute); 

    int pos = intakeDeploy.getSensorCollection().getPulseWidthPosition(); 

    intakeDeploy.getSensorCollection().setQuadraturePosition(pos-5295, 0); 

    intakeDeploy.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative); 
  }

  @Override
  public void periodic() {
    if (movingArm) {
      if (intakeDeploy.getActiveTrajectoryPosition() == targetPosition && Math.abs(intakeDeploy.getClosedLoopError()) < errorThreshold){
        withinThresholdLoops++;
      }else{
        withinThresholdLoops = 0;
      }
    }

    if(targetPosition == Constants.IntakeConstants.DEPLOY_POSITION && armInPosition()){
      stop();
    }
    // This method will be called once per scheduler run
  }

  private boolean intakeUp = true;
  public void toggleElevation(){
    intakeUp = !intakeUp;
    if(intakeUp){
      retract();
    }else{
      deploy();
    }
  }

  public boolean armInPosition() {
    return (withinThresholdLoops > loopsToSettle);
  }

  public void stop(){
    intakeDeploy.set(ControlMode.PercentOutput, 0);
    movingArm = false;
  }

  public void deploy(){
    movingArm = true;
    withinThresholdLoops = 0;
    targetPosition = Constants.IntakeConstants.DEPLOY_POSITION;
    intakeDeploy.set(ControlMode.MotionMagic, Constants.IntakeConstants.DEPLOY_POSITION);
  } 

  public void retract(){
    movingArm = true;
    withinThresholdLoops = 0;
    targetPosition = Constants.IntakeConstants.RETRACT_POSITION;
    intakeDeploy.set(ControlMode.MotionMagic, Constants.IntakeConstants.RETRACT_POSITION);
  } 

  public void enable(){
    intakeTalon.set(ControlMode.PercentOutput, Constants.IntakeConstants.SPEED);
  }

  public void disable(){
    intakeTalon.set(ControlMode.PercentOutput, 0);
  }

  public void outtake(){
    intakeTalon.set(ControlMode.PercentOutput, -Constants.IntakeConstants.SPEED);
  }
}
