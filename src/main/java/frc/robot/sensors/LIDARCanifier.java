/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import com.ctre.phoenix.CANifier;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.MeanFilter;

public class LIDARCanifier {
  /**
   * Creates a new LIDARCanifier.
   */ 
  
  CANifier canifier;
  double[] tempPWMData = new double[]{0, 0};
  
  private MeanFilter filter = new MeanFilter(Constants.lidarBufferSize);
  public LIDARCanifier(int kCanifierID) { // 5190's code told me to always pass in 16 for this value. I don't know if that's true or not, so take that info with caution.
    canifier = new CANifier(kCanifierID); 
    canifier.configFactoryDefault();
  }

  
  public double readDistance() {
    canifier.getPWMInput(CANifier.PWMChannel.PWMChannel0, tempPWMData);
    SmartDashboard.putNumber("CANifier/Channel0_0", tempPWMData[0]);
    SmartDashboard.putNumber("CANifier/Channel0_1", tempPWMData[1]);

    canifier.getPWMInput(CANifier.PWMChannel.PWMChannel1, tempPWMData);
    SmartDashboard.putNumber("CANifier/Channel1_0", tempPWMData[0]);
    SmartDashboard.putNumber("CANifier/Channel1_1", tempPWMData[1]);

    canifier.getPWMInput(CANifier.PWMChannel.PWMChannel2, tempPWMData);
    SmartDashboard.putNumber("CANifier/Channel2_0", tempPWMData[0]);
    SmartDashboard.putNumber("CANifier/Channel2_1", tempPWMData[1]);

    canifier.getPWMInput(CANifier.PWMChannel.PWMChannel3, tempPWMData);
    SmartDashboard.putNumber("CANifier/Channel3_0", tempPWMData[0]);
    SmartDashboard.putNumber("CANifier/Channel3_1", tempPWMData[1]);

    return filter.filter(tempPWMData[0]);
  }
}
