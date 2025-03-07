package frc.robot.subsystems.CoralIntake;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakeIO {
  @AutoLog
  public static class CoralIntakeIOInputs {
    public boolean hasPiece = false;
    public Distance LaserCANDistance = Millimeters.of(0);
    public boolean pivotMotorConnected = false;
    public boolean rollerMotorConnected = false;
    public double intakeVelocityRPM = 0;
    public double desiredRollerVelocityRPM = 0;
    public double desiredCarwashVelocityRPM = 0;
    public double laserCanDistanceMilimeters = 0;

    public double positionRotations = 0;
    public IntakeState state = IntakeState.RETRACT;
  }

  public default void updateInputs(CoralIntakeIOInputs inputs) {}

  public default void setSpeed(Voltage velocity) {}

  public default void setAngle(Angle angle) {}

  public default void setState(IntakeState state) {}

  public default void deploy(Angle trim) {}

  public default void retract() {}

  public default void stop() {}

  public default void zeroPosition() {}
}
