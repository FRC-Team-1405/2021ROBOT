/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

/**
 * Add your docs here.
 */
public class MathTools {
    public static int map(int x, int inMin, int inMax, int outMin, int outMax){
        return ((x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
    }

    public static double map(double x, double inMin, double inMax, double outMin, double outMax){
        return ((x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
    }
}
