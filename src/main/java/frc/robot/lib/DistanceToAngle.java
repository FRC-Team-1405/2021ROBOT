package frc.robot.lib;

import edu.wpi.first.wpilibj.util.Units;

public class DistanceToAngle {
    private static Interpolate table = new Interpolate( new Interpolate.Point[] {
        new Interpolate.Point(75.8, 0),
        new Interpolate.Point(153.0, 85000),
        new Interpolate.Point(261.0, 155000), 
        new Interpolate.Point(411.1, 200000), 
        new Interpolate.Point(562.7, 225000), 
        new Interpolate.Point(722.3, 250000)
      });
  
      /**
       * Convert a distance in centimeters to a power setting.
       * @param distance in centimeters
       * @return power value
       */
      public static double calculate(double distance){
        return table.calculate(distance);
      }
      
}
