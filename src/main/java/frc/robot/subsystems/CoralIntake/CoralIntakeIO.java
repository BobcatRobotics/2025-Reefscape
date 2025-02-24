package frc.robot.subsystems.CoralIntake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakeIO {
  @AutoLog
  public static class CoralIntakeIOInputs {
    public boolean hasPiece = false;
    public Distance LaserCANDistance = Millimeters.of(0);
    public boolean pivotMotorConnected = false;
    public boolean rollerMotorConnected = false;
    public double intakeVelocityRPS = 0;
    public double desiredRollerVelocityRPS = 0;
    public double desiredCarwashVelocityRPS = 0;

    public Angle position = Degrees.of(0);
    public IntakeState state = IntakeState.RETRACT;
  }

  public default void updateInputs(CoralIntakeIOInputs inputs) {}

  public default void setSpeed(AngularVelocity velocity) {}
  ;

  public default void setAngle(Angle angle) {}

  public default void setState(IntakeState state) {}

  public default void deploy() {}

  public default void retract() {}

  public default void stop() {}
}
