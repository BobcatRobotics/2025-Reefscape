package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class RotationUtil {

  /**
   * Wraps the angle to be within 1 positive rotation,
   *  ie 3pi = pi, -pi = pi, 450 deg = 90 deg
   */
  public static Rotation2d wrap(Rotation2d angle) {
    return Rotation2d.fromDegrees(((angle.getDegrees()%360)+360)%360);
  }

}
