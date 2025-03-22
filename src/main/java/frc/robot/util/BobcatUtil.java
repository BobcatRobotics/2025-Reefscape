package frc.robot.util;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.AngularVelocityUnit;

public class BobcatUtil {
  public static final AngularVelocityUnit RotationsPerMinute = Rotations.per(Minute);

  public static boolean isNot0(double... values) {
    for (double value : values) {
      if (value == 0) {
        return false;
      }
    }
    return true;
  }

  public static boolean isNot0(int... values) {
    for (int value : values) {
      if (value == 0) {
        return false;
      }
    }
    return true;
  }
}
