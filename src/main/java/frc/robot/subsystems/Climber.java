/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {

  /**
   * Creates a new Climber.
   */ 
  public WPI_TalonSRX leftClimbMotor = new WPI_TalonSRX(Constants.leftScissor); 
  public WPI_TalonSRX rightClimbMotor = new WPI_TalonSRX(Constants.rightScissor); 
  //Configurable values for the climb motor: 
  public double reachPosition = 190000; 
  public double lowPosition   =  35000; 
  public double homePosition = 100;
  public boolean enabled = false; 
   public int leftTargetPosition = 0;
  public int rightTargetPosition = 0;

  public Climber() {
    SmartDashboard.putBoolean("Climb Enabled", enabled); 
    SmartDashboard.putNumber("Climb Position", reachPosition); 
    SmartDashboard.putNumber("Low Position", lowPosition); 
    SmartDashboard.putNumber("Home Position", homePosition); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putBoolean("Climber Sensor Reading", leftFrontSwitch.get()); 
    leftZeroize.periodic();
    rightZeroize.periodic();
  } 

  public boolean  climbInPosition(){
    return climbInPosition_left() && climbInPosition_right();
  }

  private boolean climbInPosition_left(){
    return leftClimbMotor.getClosedLoopTarget() == leftTargetPosition
        && leftClimbMotor.getClosedLoopError(0) < 100;
  }

  private boolean climbInPosition_right(){
    return rightClimbMotor.getClosedLoopTarget() == rightTargetPosition
        && rightClimbMotor.getClosedLoopError(0) < 100 ;
}

  public void toggleEnable(){
    enabled = !enabled;
    SmartDashboard.putBoolean("Climb Enabled", enabled);
  }

  public void directControl(double left, double right){
    if(enabled){
      leftClimbMotor.set(ControlMode.PercentOutput, left);
      rightClimbMotor.set(ControlMode.PercentOutput, right);
    }
  }

  public void moveLeft(double distance){ 
    if (!leftZeroize.isZeroizeComplete() || !rightZeroize.isZeroizeComplete())
    return;
     
    if(enabled && climbInPosition_left()){
      leftClimbMotor.set(ControlMode.Position, distance);
    }
  }

  public void moveRight(double distance){ 
    if (!leftZeroize.isZeroizeComplete() || !rightZeroize.isZeroizeComplete())
    return;
     
    if(enabled && climbInPosition_right()){
      rightClimbMotor.set(ControlMode.Position, distance);
    }
  }
  //highest position
  public void reachUp(){  
    if (!leftZeroize.isZeroizeComplete() || !rightZeroize.isZeroizeComplete())
    return;
     
    if(enabled){
      int targetPosition = (int)SmartDashboard.getNumber("Climber/Climb Position", reachPosition);
      leftTargetPosition = targetPosition;
      rightTargetPosition = targetPosition;
      leftClimbMotor.set(ControlMode.MotionMagic, leftTargetPosition);
      rightClimbMotor.set(ControlMode.MotionMagic, rightTargetPosition);
    }
  } 
  //45"
  public void reachLow(){   
    if (!leftZeroize.isZeroizeComplete() || !rightZeroize.isZeroizeComplete())
    return;
     
    int targetPosition = (int)SmartDashboard.getNumber("Low Position", lowPosition);
    rightClimbMotor.set(ControlMode.MotionMagic, targetPosition); 
    leftClimbMotor.set(ControlMode.MotionMagic, targetPosition);
  }
  //lowest postition
  public void goHome(){
    if (!leftZeroize.isZeroizeComplete() || !rightZeroize.isZeroizeComplete())
      return ;

    int targetPosition = (int)SmartDashboard.getNumber("Home Position", homePosition);
    leftClimbMotor.set(ControlMode.MotionMagic, targetPosition);
    rightClimbMotor.set(ControlMode.MotionMagic, targetPosition);
  } 

  private class ZeroizeState{
    ZeroizeState(WPI_TalonSRX motor){
      this.motor = motor;
    }
    public void periodic(){
      if (!zeroizeActive) {
        if (motor.isRevLimitSwitchClosed() == 1){
          motor.set(ControlMode.PercentOutput, 0);
          zeroizeActive = false;
          zeroizeComplete = true;
        }
      }
    }
    public void zeroize(){
      if (!zeroizeActive){
        zeroizeActive = true;
        motor.set(ControlMode.PercentOutput, 0.25); 
      } 
    }
    public boolean isZeroizeComplete(){
      return zeroizeComplete;
    }
    private boolean zeroizeActive = false; 
    private boolean zeroizeComplete = false; 
    private WPI_TalonSRX motor;
  }

  private ZeroizeState leftZeroize = new ZeroizeState(leftClimbMotor);
  private ZeroizeState rightZeroize = new ZeroizeState(rightClimbMotor);

  public void zeroize(){ 
    leftZeroize.zeroize();
    rightZeroize.zeroize();
  } 

  // get lengths in meters
  public double getLeftScissorLength(){
    return 0.0;
  }

  public double getRightScissorLength(){
    return 0.0;
  }
  public void resetClimberEncoders(){
    leftClimbMotor.setSelectedSensorPosition(0);
    rightClimbMotor.setSelectedSensorPosition(0);
  }
}
