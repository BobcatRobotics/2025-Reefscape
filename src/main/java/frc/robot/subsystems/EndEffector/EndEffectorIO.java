package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {
    public AngularVelocity velocity = RotationsPerSecond.of(0);
    public double laserCanDistanceMilimeters = -1;
    public double currentDraw = -1;
    public boolean hasPiece = false;
    public boolean motorConnected = false;
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setSpeed(double speed) {}
}
