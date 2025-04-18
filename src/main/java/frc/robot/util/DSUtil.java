package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DSUtil {

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
}
