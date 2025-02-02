package frc.robot.subsystems.CoralIntake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class CoralIntake {
  private boolean deployed;

  /**
   * Distance between minimum elevator position and top of the coral intake
   */
  public static final Distance CORAL_INTAKE_HEIGHT = Meters.of(0);

  public CoralIntake() {}

  public void setState(IntakeState state) {}

  public boolean deployed() {
    return deployed;
  }
}
