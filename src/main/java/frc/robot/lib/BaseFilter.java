/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;
import java.util.Arrays;

public abstract class BaseFilter {
    double values[];
    protected int position = 0; 
    protected boolean isInitialized = false;

    public BaseFilter(int size){
        values = new double[size];
        position = 0;
    }

    public void reset(){
        isInitialized = false;
    }

    public double filter(double value){
        if (!isInitialized){
            Arrays.fill(values, value);
            position = 0;
            isInitialized = true; 
        }
        
        values[position++ % values.length] = value;
        
        return calculation();
    }

    public abstract double calculation();
}
