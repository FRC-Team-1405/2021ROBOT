package frc.robot.lib;

import edu.wpi.first.wpilibj.util.Units;

public class DistanceToAngle {
    private static Interpolate table = new Interpolate( new Interpolate.Point[] {
        new Interpolate.Point(Units.feetToMeters(0) *100.0, 10000),
        new Interpolate.Point(Units.feetToMeters(10)*100.0, 180000),
        new Interpolate.Point(Units.feetToMeters(20)*100.0, 350000)
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
