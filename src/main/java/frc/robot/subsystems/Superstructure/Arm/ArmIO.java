package frc.robot.subsystems.Superstructure.Arm;

import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    public Rotation2d absolutePosition = new Rotation2d();
    public ArmState state = ArmState.RIGHT_SIDE_UP;
    // public ArmZone zone = ArmZone.BOTTOM_ZONE;
    public boolean motorConnected = false;
    public boolean encoderConnected = false;
    public double torqueCurrentAmps = -1;
    public double velocityRotPerSec = -1;
    public boolean aligned = false;
    public double positionRotations = 0;
    public ControlModeValue controlMode = ControlModeValue.DisabledOutput;
    public double desiredPositionRotation = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setDesiredState(ArmState state) {}
}
