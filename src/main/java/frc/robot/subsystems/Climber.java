/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
  public double reachPosition = 155000; 
  public double lowPosition   =  100000; 
  public double homePosition = 100;
  public double moveRange = 10000;
  public boolean enabled = false; 
  private static final double ZEROIZE_SPEED = -0.4;

  private enum Zeroize {
    Initialize{
      public Zeroize execute(TalonSRX motor) {
        motor.set(ControlMode.PercentOutput, ZEROIZE_SPEED);
        return Zeroize;
      };
    },
    Zeroize{
      public Zeroize execute(TalonSRX motor) {
        if (motor.isRevLimitSwitchClosed() == 1){
          motor.set(ControlMode.PercentOutput, 0);
          return Ready;
        }
        return this;
      };
    },
    Ready{
      public Zeroize execute(TalonSRX motor) {
        return this;
      };
    };
    public abstract Zeroize execute(TalonSRX motor);
  };

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
    leftZeroize = leftZeroize.execute(leftClimbMotor);
    rightZeroize = rightZeroize.execute(rightClimbMotor);
  } 

  public boolean  climbInPosition(){
    return climbInPosition_left() && climbInPosition_right();
  }

  private boolean climbInPosition_left(){
    return leftClimbMotor.getClosedLoopError(0) < 3000;
  }

  private boolean climbInPosition_right(){
    return rightClimbMotor.getClosedLoopError(0) < 3000;
}

  public void toggleEnable(){
    enabled = !enabled;
    SmartDashboard.putBoolean("Climb Enabled", enabled);
  }

  public void directControl(double left, double right){
    if (leftZeroize != Zeroize.Ready || rightZeroize != Zeroize.Ready)
      return ;

    if(enabled){
      leftClimbMotor.set(ControlMode.PercentOutput, left);
      rightClimbMotor.set(ControlMode.PercentOutput, right);
    }
  }

  public void stop(){
    leftClimbMotor.set(ControlMode.MotionMagic, leftClimbMotor.getSelectedSensorPosition());
    rightClimbMotor.set(ControlMode.MotionMagic, rightClimbMotor.getSelectedSensorPosition());
  }

  public void moveLeft(double distance){ 
    if (leftZeroize != Zeroize.Ready || rightZeroize != Zeroize.Ready)
      return ;
     
    if(enabled && climbInPosition_left()){
      int target = (int) leftClimbMotor.getSelectedSensorPosition() + (int)(distance * moveRange);
      leftClimbMotor.set(ControlMode.MotionMagic, target);
    }
  }

  public void moveRight(double distance){ 
    if (leftZeroize != Zeroize.Ready || rightZeroize != Zeroize.Ready)
      return ;
     
    if(enabled && climbInPosition_right()){
      int target = (int) rightClimbMotor.getSelectedSensorPosition() + (int)(distance * moveRange);
      rightClimbMotor.set(ControlMode.MotionMagic, target);
    }
  }
  //highest position
  public void reachUp(){  
    if (leftZeroize != Zeroize.Ready || rightZeroize != Zeroize.Ready)
      return ;
     
    if(enabled){
      int targetPosition = (int)SmartDashboard.getNumber("Climber/Climb Position", reachPosition);
      leftClimbMotor.set(ControlMode.MotionMagic, targetPosition);
      rightClimbMotor.set(ControlMode.MotionMagic, targetPosition);
    }
  } 
  //45"
  public void reachLow(){   
    if (leftZeroize != Zeroize.Ready || rightZeroize != Zeroize.Ready)
      return ;
     
    if(enabled){
      int targetPosition = (int)SmartDashboard.getNumber("Climber/Low Position", lowPosition);
      rightClimbMotor.set(ControlMode.MotionMagic, targetPosition); 
      leftClimbMotor.set(ControlMode.MotionMagic, targetPosition);
    }
  }

  //lowest postition
  public void goHome(){
    if (leftZeroize != Zeroize.Ready || rightZeroize != Zeroize.Ready)
      return ;

    int targetPosition = (int)SmartDashboard.getNumber("Climber/Home Position", homePosition);
    leftClimbMotor.set(ControlMode.MotionMagic, targetPosition);
    rightClimbMotor.set(ControlMode.MotionMagic, targetPosition);
  } 

  private Zeroize leftZeroize = Zeroize.Ready;  // disable auto zeroize
  private Zeroize rightZeroize = Zeroize.Ready; // disable auto zeroize

  public void zeroize(){ 
    leftZeroize = Zeroize.Initialize;
    rightZeroize = Zeroize.Initialize;
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
