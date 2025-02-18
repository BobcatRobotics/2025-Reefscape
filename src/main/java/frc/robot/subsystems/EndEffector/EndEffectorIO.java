package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {
    public double rpm = -1;
    public double laserCanDistanceMilimeters = -1;
    public double currentDraw = -1;
    public boolean hasPiece = false;
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setSpeed(double speed) {}
}
