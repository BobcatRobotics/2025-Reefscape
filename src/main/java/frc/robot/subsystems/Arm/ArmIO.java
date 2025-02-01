package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    Rotation2d position = new Rotation2d();
    ArmState state = ArmState.NO_OP;
    ArmZone zone = ArmZone.BOTTOM_ZONE;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setState(ArmState state){}
}
