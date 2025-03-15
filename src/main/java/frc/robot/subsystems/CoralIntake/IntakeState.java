package frc.robot.subsystems.CoralIntake;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public enum IntakeState {
  DEPLOY(13.5), // TODO find this
  RETRACT(0.74),
  ALGAE_PICKUP(8),
  UNKNOWN(-1);

  IntakeState(double rotations) {
    this.angle = Rotations.of(rotations);
  }

  public Angle angle;
}
