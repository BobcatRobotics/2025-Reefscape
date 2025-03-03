package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {
    public double velocityRPS = 0;
    public double laserCanDistanceMilimeters = -1;
    public double currentDraw = -1;
    public boolean hasPiece = false;
    public boolean motorConnected = false;
    public double desiredSpeedRPS = 0;
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setSpeed(double speed) {}
}
