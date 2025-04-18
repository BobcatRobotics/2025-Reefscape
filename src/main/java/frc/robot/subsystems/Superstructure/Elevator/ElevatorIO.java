package frc.robot.subsystems.Superstructure.Elevator;

import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public ElevatorState state = ElevatorState.UNKNOWN;
    public Rotation2d rotPosition = Rotation2d.kZero;
    public double positionRotations = 0;
    public double velocityRotPerSec = -1;
    public double positionPercent = -1;
    public double heightMeters = -1;
    /** is the elevator at its desired state? */
    public boolean aligned = false;

    public boolean overriden = false;
    public double distanceToAlignment = 0;

    public ControlModeValue controlMode = ControlModeValue.DisabledOutput;

    public boolean motorConnected = false;
    public boolean encoderConnected = false;

    public double appliedVolts = 0;
    public double supplyCurrentAmps = 0;

    public double closedLoopReferenceSlope = 0;
    public double closedLoopReference = 0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setDesiredState(ElevatorState state) {}

  public default void manualOverride(double percent) {}
}
