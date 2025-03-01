package frc.robot.subsystems.Climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double positionRotations = -1;
    public boolean connected = false;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setDutyCycle(double output) {}

  public default void setPosition(Rotation2d pos) {}
}
