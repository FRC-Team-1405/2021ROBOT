package frc.robot.lib;

/** Add your docs here. */
public class DistanceToPower { 
  private static Interpolate table = new Interpolate( new Interpolate.Point[] {
    new Interpolate.Point(203.0,  8000),
    new Interpolate.Point(303.0,  9000),
    new Interpolate.Point(402.0,  9500), 
    new Interpolate.Point(497.0, 10000), 
    new Interpolate.Point(589.0, 11000), 
    new Interpolate.Point(710.0, 12000)
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