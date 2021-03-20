/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class SmartSupplier implements DoubleSupplier{
    private String key;
    private double defaultValue;

    public SmartSupplier(String key, double defaultValue){
        this.key = key;
        this.defaultValue = defaultValue;
        SmartDashboard.setPersistent(key);
        if (!SmartDashboard.containsKey(key)){
            SmartDashboard.putNumber(key, defaultValue);
        }
    }

    @Override
	public double getAsDouble() {
        return SmartDashboard.getNumber(this.key, this.defaultValue);
     }
}
