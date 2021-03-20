package frc.robot.lib.thirdcoast.talon;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import java.util.logging.Logger;

/** Utility class to check for and display TalonSRX configuration errors. */
public class Errors {

  private static boolean summarized = true;
  private static int count;

  public static void check(ErrorCode error, Logger logger) {
    if (error != null && error != ErrorCode.OK) {
      if (summarized) count++;
      else logger.severe(String.format("error while configuring Talon: %s", error));
    }
  }

  public static void check(TalonSRX talon, String method, ErrorCode error, Logger logger) {
    if (error != null && error != ErrorCode.OK) {
      if (summarized) count++;
      else logger.severe(String.format("Talon %d: %s error %s", talon.getDeviceID(), method, error));
    }
  }

  public static boolean isSummarized() {
    return summarized;
  }

  public static void setSummarized(boolean summarized) {
    Errors.summarized = summarized;
  }

  public static int getCount() {
    return count;
  }

  public static void setCount(int count) {
    Errors.count = count;
  }
}
