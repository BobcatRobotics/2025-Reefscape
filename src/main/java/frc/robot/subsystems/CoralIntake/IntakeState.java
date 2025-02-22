package frc.robot.subsystems.CoralIntake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public enum IntakeState {
  DEPLOY(210), // TODO find this
  RETRACT(0),
  UNKNOWN(-1);

  IntakeState(double degrees) {
    this.angle = Degrees.of(degrees);
  }

  public Angle angle;
}
