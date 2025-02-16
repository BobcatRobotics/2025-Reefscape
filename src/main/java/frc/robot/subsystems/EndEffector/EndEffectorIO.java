package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {
    double rpm = -1;
    double laserCanDistanceMeters = -1;
    double currentDraw = -1;
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setSpeed(double speed) {}
}
