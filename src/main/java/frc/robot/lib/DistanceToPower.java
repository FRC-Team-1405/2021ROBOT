package frc.robot.lib;

/** Add your docs here. */
public class DistanceToPower { 
  private static Interpolate table = new Interpolate( new Interpolate.Point[] {
    new Interpolate.Point(75.8, 12000),
    new Interpolate.Point(153.0, 12000),
    new Interpolate.Point(261.0, 12000), 
    new Interpolate.Point(411.1, 12000), 
    new Interpolate.Point(562.7, 12000), 
    new Interpolate.Point(722.3, 14000)
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