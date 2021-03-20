/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.sensors.ColorSensor;
import frc.robot.sensors.FMSData;

public class ControlPanel extends SubsystemBase {
  /**
   * Creates a new ControlPanel.
   */

  WPI_TalonSRX controlMotor = new WPI_TalonSRX(Constants.controlPanel);
  Color[] colors = new Color[] {ColorSensor.Target.BLUE, ColorSensor.Target.GREEN, ColorSensor.Target.RED, ColorSensor.Target.YELLOW,};
  //CANSparkMax controlMotor = new CANSparkMax(Constants.controlPanel, MotorType.kBrushless);

  private ColorSensor sensor = new ColorSensor();
  public ControlPanel() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean checkColor(){
    return FMSData.getColor() == sensor.readColor();
  }

  public void rotationControl(double distance){
    controlMotor.set(ControlMode.Position, distance);
  }

  public void stop() {
    controlMotor.set(ControlMode.PercentOutput, 0);
  }

  private static final int MIN_POSITION_ERROR = 1000;
  public boolean isRotationComplete(){
    return controlMotor.getClosedLoopError() < MIN_POSITION_ERROR;
  }
 
  public void positionControl(){
    controlMotor.set(ControlMode.PercentOutput, Constants.ControlPanelConstants.SPEED);
  }

  public double findDistance(Color currentColor, Color targetColor) throws Exception {
    // TODO get an offset for future values
    // TODO get absolute turnWheelRadius
    int currentIndex = -1, targetIndex = -1;
    for (int i = 0; i < colors.length; i++) {
      if (colors[i] == currentColor) {
        currentIndex = i;
        break;
      }
    }
    for (int i = 0; i < colors.length; i++) {
      if (colors[i] == targetColor) {
        targetIndex = i;
        break;
      }
    }
    if (currentIndex == -1 || targetIndex == -1) { throw new Exception("Target/Current value not defined"); }
    //        find relative distance needed * length of one segment
    double distance = (targetIndex - currentIndex) * Constants.ControlPanelConstants.ROTATION_SEGMENT_DISTANCE;
    if(!Robot.fmsAttached){
      SmartDashboard.putNumber("ControlPanel/turnDistance", distance);
    }
    return distance;
  }
}
