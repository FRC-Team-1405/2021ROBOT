package frc.robot.lib;

import edu.wpi.first.wpilibj.util.Units;

/** Add your docs here. */
public class DistanceToPower { 
    private static Interpolate table = new Interpolate( new Interpolate.Point[] {
      new Interpolate.Point(Units.feetToMeters(1.0) *100.0, 1000),
      new Interpolate.Point(Units.feetToMeters(15.0)*100.0, 1000),
      new Interpolate.Point(Units.feetToMeters(20.0)*100.0, 15000)
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