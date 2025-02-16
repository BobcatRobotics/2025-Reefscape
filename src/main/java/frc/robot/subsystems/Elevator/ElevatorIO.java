package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    ElevatorState state = ElevatorState.UNKNOWN;
    Rotation2d rotPosition = Rotation2d.kZero;
    double velocityRotPerSec = -1;
    double torqueCurrentAmps = -1;
    double positionPercent = -1;
    /**is the elevator at its desired state?*/
    boolean aligned = false;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setDesiredState(ElevatorState state) {}

  public default boolean inTolerance(){return false;}
}
