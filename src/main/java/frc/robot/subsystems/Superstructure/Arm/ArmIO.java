package frc.robot.subsystems.Superstructure.Arm;

import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    public Rotation2d absolutePosition = new Rotation2d();
    public ArmState state = ArmState.RIGHT_SIDE_UP;
    public boolean motorConnected = false;
    public boolean encoderConnected = false;
    public double velocityRotPerSec = -1;
    public boolean aligned = false;
    public double positionDegrees = 0;
    public ControlModeValue controlMode = ControlModeValue.DisabledOutput;
    public double desiredPositionDegrees = 0;
    public double distanceToAlignment = 0;
    public boolean flipped = false;
    public boolean isOverridden = false;
    public double appliedVolts = 0;
    public double appliedCurrent = 0;
    /** motion magic desired velocity setpoint */
    public double closedLoopReferenceSlope = 0;
    /** motion magic desired position reference */
    public double positionReference = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setDesiredState(ArmState state, boolean flipped, boolean hasPiece) {}

  public default void manualOverride(double percent) {}
}
