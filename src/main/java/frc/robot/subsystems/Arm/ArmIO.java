package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    public Rotation2d position = new Rotation2d();
    public Rotation2d absolutePosition = new Rotation2d();
    public ArmState state = ArmState.RIGHT_SIDE_UP;
    public ArmZone zone = ArmZone.BOTTOM_ZONE;
    public boolean motorConnected = false;
    public boolean encoderConnected = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setDesiredState(Rotation2d rot) {}
}
