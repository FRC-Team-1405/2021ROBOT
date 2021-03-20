/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.sensors.LidarLitePWM;
import frc.robot.sensors.Limelight;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */ 
    private class Setting {
      public Setting(double pow, double dis) {
        power = pow;
        distance = dis;
      }
      public double power;
      public double distance;
    } ; 

  public WPI_TalonSRX left = new WPI_TalonSRX(Constants.shooterLeft); 
  public WPI_TalonSRX right = new WPI_TalonSRX(Constants.shooterRight); 

  public WPI_TalonSRX hoodLeft = new WPI_TalonSRX(27); 
  public WPI_TalonSRX hoodRight = new WPI_TalonSRX(28);
  
  public WPI_TalonSRX turret = new WPI_TalonSRX(Constants.turretid);
  public WPI_TalonSRX indexer = new WPI_TalonSRX(Constants.indexerid);
  public WPI_TalonSRX trigger = new WPI_TalonSRX(Constants.triggerid);
  public Servo leftActuator = new Servo(Constants.leftActuatorId);
  public Servo rightActuator = new Servo(Constants.rightActuatorId);
  
  private Limelight limelight;

  // public CANEncoder leftEncoder = new CANEncoder(left); 
  // public CANEncoder rightEncoder = new CANEncoder(right); 

  // CANPIDController leftPIDController = new CANPIDController(left);  
  // CANPIDController rightPIDController = new CANPIDController(right);
  
  public final LidarLitePWM lidarLitePWM = new LidarLitePWM(new DigitalInput(9)); 
  public double triggerSpeed = 0.6; 
  private int errorThreshold = 25;
  private int loopsToSettle = 10;
  private int withinThresholdLoops = 0;
  private int targetPosition = Constants.ShooterConstants.unitsMin;
  public boolean tracking = false;

  public Shooter(Limelight limelight) { 
    this.limelight = limelight;

    SmartDashboard.putNumber("Trigger Speed", triggerSpeed); 

    turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    int pos = turret.getSensorCollection().getPulseWidthPosition(); 
    turret.getSensorCollection().setQuadraturePosition(pos-3623, 0); 
    
    turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 

    SmartDashboard.putNumber("Turret Position", turret.getSelectedSensorPosition());
   
    
    
    if (turret.getActiveTrajectoryPosition() == targetPosition && Math.abs(turret.getClosedLoopError()) < errorThreshold){
      withinThresholdLoops++;
    }else{
      withinThresholdLoops = 0;
    } 

    SmartDashboard.putNumber("Lidar_Distance", lidarLitePWM.getDistance());

    if(!Robot.fmsAttached){
      SmartDashboard.putNumber("Left Error", left.getClosedLoopError()); 
      SmartDashboard.putNumber("Right Error", right.getClosedLoopError()); 
      SmartDashboard.putNumber("Limelight/TXPos", limelight.getTXPos());
      SmartDashboard.putNumber("Limelight/TYPos", limelight.getTYPos());
      SmartDashboard.putNumber("Lidar_Distance", lidarLitePWM.getDistance());
    }else{
      //SmartDashboard.putNumber("Lidar_Distance", lidarLitePWM.getDistance());
    }
  }

  public void launch(double leftDistance, double rightDistance){
    // leftPIDController.setReference(leftDistance, ControlType.kVelocity);
    // rightPIDController.setReference(rightDistance, ControlType.kVelocity); 
    left.set(ControlMode.Velocity, leftDistance); 
    right.set(ControlMode.Velocity, rightDistance); 
    if(!Robot.fmsAttached){
      SmartDashboard.putNumber("Left Distance", leftDistance); 
      SmartDashboard.putNumber("Right Distance", rightDistance); 
    }
  }

  public void stopFlywheels(){
    tracking = false;
    limelight.setPipeline((byte) 0);
    limelight.setLED((byte) 1);
    left.set(ControlMode.PercentOutput, 0.0); 
    right.set(ControlMode.PercentOutput, 0.0); 
    trigger.set(ControlMode.PercentOutput, 0);
  }

  public boolean isReady(){
    boolean ready = SmartDashboard.getBoolean("Shooter/isReady", false);    
    if (ready){
      SmartDashboard.putString("Shooter/launch", "");
    }
    return ready;
  }

  private double CalculatePower(double distance, Setting low, Setting high){ 
    //From slope point formula (y-y_1) = m(x-x_1) -> y = m(x-x_1) + y_1
    double power = (((high.power - low.power) / (high.distance - low.distance)) * (distance - low.distance) + low.power);
    return power;
  }

  Setting settings[] = new Setting[] {
                                        new Setting(10000, 1.82),
                                        new Setting(10500, 0.97),
                                        new Setting(11000, 0.625), };

  public void prepFlywheels(){
    limelight.setPipeline((byte) 7);
    limelight.setLED((byte) 3);
    tracking = true;
    turnTurret();
    int lowIndex = 0;
    int highIndex = 0;
    double power = 0.0;
    double distance = limelight.getTA();

    if(distance <= settings[0].distance){
      power = settings[0].power;
    }
    else if(distance >= settings[settings.length-1].distance){
      power = settings[settings.length-1].power;
    }
    else{
      for(int i = 1; i < settings.length; i++){
        if(distance <= settings[i].distance){
          lowIndex = i-1;
          highIndex = i;
          power = CalculatePower(distance, settings[lowIndex], settings[highIndex]);
          break;
        }
      }
    }
    left.set(ControlMode.Velocity, -power - RobotContainer.increase);
    right.set(ControlMode.Velocity, power + RobotContainer.increase);
    trigger.set(ControlMode.PercentOutput, -0.6);
    //trigger.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Trigger Speed", triggerSpeed));
  }

  public void prepFlywheels(DoubleSupplier leftV, DoubleSupplier rightV){
    limelight.setPipeline((byte) 7);
    limelight.setLED((byte) 3);
    tracking = true;
    turnTurret();
    prepFlywheels(leftV.getAsDouble() - RobotContainer.increase, rightV.getAsDouble() + RobotContainer.increase);
  } 

  public void prepFlywheels(double leftV, double rightV){
    tracking = true;
    left.set(ControlMode.Velocity, -leftV - RobotContainer.increase);
    right.set(ControlMode.Velocity, rightV + RobotContainer.increase);
    trigger.set(ControlMode.PercentOutput, -0.6);
  }

  public void fire(){
    indexer.set(ControlMode.PercentOutput, 0.6);

  }

  public void stopIndexer(){
    indexer.set(ControlMode.PercentOutput, 0);
    //trigger.set(0);
  }

  public void outtake(){
    indexer.set(ControlMode.PercentOutput, -0.5);
    //trigger.set(ControlMode.PercentOutput, 0.3);
  }

  public boolean flywheelReady() {
    return (Math.abs(left.getClosedLoopError()) <= Constants.maxFlywheelError && Math.abs(right.getClosedLoopError()) <= Constants.maxFlywheelError);
  }

  public boolean setLow = false;

  public void toggleElevation(){
    setLow = !setLow;
    setElevation();
  }

  public void setElevationManual(double elevation){
    leftActuator.set(elevation);
    rightActuator.set(elevation);
  }

  public void setElevation(){
    if(setLow){
      leftActuator.set(Constants.ShooterConstants.elevationMin);
      rightActuator.set(Constants.ShooterConstants.elevationMin);
    }else{
      leftActuator.set(Constants.ShooterConstants.elevationMax); 
      rightActuator.set(Constants.ShooterConstants.elevationMax);
    }

  }

  public boolean hasTarget(){
    return limelight.hasTarget();
  }

  public void turnToGoal(ArcadeDrive driveBase){
    withinThresholdLoops = 0;
    Pose2d pose = driveBase.getPose();
    double currentX = pose.getTranslation().getX();
    double currentY = pose.getTranslation().getY();
    double heading = pose.getRotation().getDegrees();
    double angle = 90.0 - Math.atan((-currentY)/(Constants.goalX - currentX)) - heading;
    this.turnTurret((int) angle);
  }

  public boolean turretTurnIsComplete(){
    return (withinThresholdLoops > loopsToSettle);
    //return turret.getClosedLoopError() < Constants.ShooterConstants.turretError;
  }
  public void turnTurret(){
    withinThresholdLoops = 0;
    if(limelight.getPipeline() == 7){
      turnTurret(-(int) limelight.getTX());
    }
  };

  public void goHome(){
    turret.set(ControlMode.MotionMagic, Constants.ShooterConstants.turretCenter);
  }

  public void resetEncoder(){
    turret.setSelectedSensorPosition(0);
  }

  public void turnTurret(double angle){
    int currentPos = turret.getSelectedSensorPosition();
    angle = (int) (angle * Constants.ShooterConstants.unitsPerAngle) * 
    1.15;
    if(currentPos+angle < Constants.ShooterConstants.unitsMin){
      targetPosition = Constants.ShooterConstants.unitsMin;
      turret.set(ControlMode.MotionMagic, Constants.ShooterConstants.unitsMin);
    } else if(currentPos+angle > Constants.ShooterConstants.unitsMax){
      targetPosition = Constants.ShooterConstants.unitsMax;
      turret.set(ControlMode.MotionMagic, Constants.ShooterConstants.unitsMax);
    } else{
      turret.set(ControlMode.MotionMagic, currentPos+angle);
      targetPosition = (int) (currentPos+angle);
    }
  }

  public boolean turretReady(){
    return limelight.hasTarget() && (Math.abs(limelight.getTX()) <= Constants.ShooterConstants.limelightError);
  }

  public void stopTurret(){
    turret.set(ControlMode.Position, turret.getSelectedSensorPosition());
  }
}
