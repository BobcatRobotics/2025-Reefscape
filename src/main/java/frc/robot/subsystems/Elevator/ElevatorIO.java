package frc.robot.subsystems.Elevator;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    Distance position;
    ElevatorState state;
  }

  public default void updateInputs() {}

  public default void setDesiredState() {}
}
