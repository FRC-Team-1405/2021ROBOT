package frc.robot.lib;

public class DistanceToAngle {
    private static Interpolate table = new Interpolate( new Interpolate.Point[] {
      new Interpolate.Point(75.8, 0),
      new Interpolate.Point(153.0, 10000),
      new Interpolate.Point(261.0, 15000), 
      new Interpolate.Point(411.1, 20000), 
      new Interpolate.Point(562.7, 32500), 
      new Interpolate.Point(722.3, 34000)
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
