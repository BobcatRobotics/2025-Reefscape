package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    Distance position;
    ElevatorState state;
    Rotation2d rotPosition;
    double velocityRadPerSec;
    double appliedVolts;
    double torqueCurrentAmps;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setDesiredState(ElevatorState state) {}

  public default void runOpenLoop(double output) {}

  public default void runVolts(double volts) {}

  public default void stop() {}

  public default void runPosition(double positionRad, double feedforward) {}

  public default void setPID(double kP, double kI, double kD) {}
}
