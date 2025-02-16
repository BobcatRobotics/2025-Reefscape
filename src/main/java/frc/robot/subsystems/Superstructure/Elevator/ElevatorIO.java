package frc.robot.subsystems.Superstructure.Elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.ControlModeValue;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    ElevatorState state = ElevatorState.UNKNOWN;
    Rotation2d rotPosition = Rotation2d.kZero;
    double velocityRotPerSec = -1;
    double torqueCurrentAmps = -1;
    double positionPercent = -1;
    /** is the elevator at its desired state? */
    boolean aligned = false;
    ControlModeValue controlMode = ControlModeValue.DisabledOutput;

    boolean motorConnected = false;
    boolean encoderConnected = false;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setDesiredState(ElevatorState state) {}

  public default void runVoltage(Voltage volts) {}
}
