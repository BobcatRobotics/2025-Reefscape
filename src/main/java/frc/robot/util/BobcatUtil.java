package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import frc.lib.BobcatLib.CANdle.BuiltInAnimations;
// import frc.robot.Constants.CANdleConstants;

public class BobcatUtil {
  public static Alliance getAlliance() {
    return DriverStation.getAlliance().isEmpty()
        ? Alliance.Blue
        : DriverStation.getAlliance().get();
  }

  public static boolean isBlue() {
    return getAlliance() == Alliance.Blue;
  }

  public static boolean isRed() {
    return getAlliance() == Alliance.Red;
  }

  public static double get0to2Pi(double rad) {
    rad = rad % (2 * Math.PI);
    if (rad < (0)) {
      rad += (2 * Math.PI);
    } // should this be here?
    return rad;
  }

  public static double get0to2Pi(Rotation2d rot) {
    return get0to2Pi(rot.getRadians());
  }
  /**
   * wraps the rotation2d to be within one rotation, i.e. a rotation2d with a value of 370 degrees
   * will return a rotation2d with a value of 10 degrees
   */
  public static Rotation2d wrapRot2d(Rotation2d rot) {
    return Rotation2d.fromRadians(get0to2Pi(rot));
  }
}
