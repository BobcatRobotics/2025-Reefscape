package frc.robot.subsystems.CoralIntake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public enum IntakeState {
  DEPLOY(13.5, Units.radiansToDegrees(-8.5)), // TODO find this
  RETRACT(0.85, 90),
  ALGAE_PICKUP(8, 100),
  UNKNOWN(-1, -1);

  IntakeState(double rotations, double simDegrees) {
    this.angle = Rotations.of(rotations);
    this.simAngle = Degrees.of(simDegrees);
  }

  public Angle angle;
  public Angle simAngle;
}
