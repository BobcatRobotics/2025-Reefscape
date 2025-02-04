package frc.robot.subsystems.AlgaeIntake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.CoralIntake.IntakeState;

public class AlgaeIntake {
  private boolean deployed;
  /** Distance between minimum elevator position and top of the coral intake */
  public static final Distance ALGAE_INTAKE_HEIGHT = Meters.of(0);

  public AlgaeIntake() {}

  public void setState(IntakeState state) {}

  public boolean deployed() {
    return deployed;
  }
}
