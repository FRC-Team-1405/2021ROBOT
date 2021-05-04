// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.wpilibj.util.Units;

/** Add your docs here. */
public class DistanceToPower { 

    private static class Point {
        public final double distance; 
        public final double power; 

        public Point(double distance, double power){ 
            this.distance = distance;
            this.power = power;     
        }
    }

    private static Point[] points = new Point[] { new Point(Units.feetToMeters(1.0) *100.0, 1000),
                                                  new Point(Units.feetToMeters(15.0)*100.0, 1000),
                                                  new Point(Units.feetToMeters(20.0)*100.0, 15000)
                                                };

    public static double calculatePower(double distance){
        int lowIndex = 0;
        int highIndex = 0;
        double power = 0.0;
        
    
        if(distance <= points[0].distance){
          power = points[0].power;
        }
        else if(distance >= points[points.length-1].distance){
          power = points[points.length-1].power;
        }
        else{
          for(int i = 1; i < points.length; i++){
            if(distance <= points[i].distance){
              lowIndex = i-1;
              highIndex = i;
              power = Interpolate(distance, points[lowIndex], points[highIndex]);
              break;
            }
          }
        }
      return power;
    }

    private static double Interpolate(double distance, Point low, Point high){ 
      //From slope point formula (y-y_1) = m(x-x_1) -> y = m(x-x_1) + y_1
      double power = (((high.power - low.power) / (high.distance - low.distance)) * (distance - low.distance) + low.power);
      return power;
    }
}