/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib; 

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Add your docs here.
 */
public class SmartBooleanSupplier implements BooleanSupplier {
    private String key;
    private boolean defaultValue;

    public SmartBooleanSupplier(String key, boolean defaultValue){
        this.key = key;
        this.defaultValue = defaultValue; 
        if(!SmartDashboard.containsKey(key)){
            SmartDashboard.putBoolean(key, defaultValue);
        }
    }

    @Override
	public boolean getAsBoolean() {
        return SmartDashboard.getBoolean(this.key, this.defaultValue);
     }
}
