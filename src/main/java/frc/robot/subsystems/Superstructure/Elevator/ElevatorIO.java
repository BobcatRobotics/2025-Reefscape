package frc.robot.subsystems.Superstructure.Elevator;

import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    ElevatorState state = ElevatorState.UNKNOWN;
    Rotation2d rotPosition = Rotation2d.kZero;
    double positionRotations = 0;
    double velocityRotPerSec = -1;
    double torqueCurrentAmps = -1;
    double positionPercent = -1;
    double heightMeters = -1;
    /** is the elevator at its desired state? */
    boolean aligned = false;
    boolean overriden = false;
    double distanceToAlignment = 0;

    ControlModeValue controlMode = ControlModeValue.DisabledOutput;

    boolean motorConnected = false;
    boolean encoderConnected = false;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setDesiredState(ElevatorState state) {}

  public default void manualOverride(double percent) {}
}
