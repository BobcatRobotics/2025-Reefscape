package frc.robot.subsystems.Superstructure.Elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.RobotVisualizer;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  public static final Distance DRUM_RADIUS =
      Inches.of(0.937082); // I have no idea if this is correct
  public static final Rotation2d ELEVATOR_TOLERANCE =
      // Rotation2d.fromDegrees(125); // should be about 1 (?) in of travel
      Rotation2d.fromDegrees(75);

  /** the number of rotations the encoder spins when it is at the top */
  public static final Rotation2d MAX_ROTATIONS = Rotation2d.fromRadians(25.8);

  public static final Distance MAX_HEIGHT = Inches.of(69.344772); // from cad

  /** the number of output shaft rotations per meter of elevator travel */
  public static final double ROTATIONS_PER_METER =
      MAX_ROTATIONS.getRotations() / MAX_HEIGHT.baseUnitMagnitude();

  public static final double METERS_PER_ROTATION = 1 / ROTATIONS_PER_METER;

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert =
      new Alert("Elevator motor disconnected!", Alert.AlertType.kWarning);
  private final Alert encoderDisconnectedAlert =
      new Alert("Elevator encoder disconnected!", Alert.AlertType.kWarning);

  private RobotVisualizer visualizer = RobotVisualizer.getInstance();

  private ElevatorState desiredState = ElevatorState.UNKNOWN;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    motorDisconnectedAlert.set(!inputs.motorConnected);
    encoderDisconnectedAlert.set(!inputs.encoderConnected);
    visualizer.setElevatorHeight(Meters.of(inputs.heightMeters));
  }

  public double positionPercent() {
    return inputs.positionPercent;
  }

  public void setState(ElevatorState desiredState) {
    this.desiredState = desiredState;
    io.setDesiredState(desiredState);
  }

  public ElevatorState getState() {
    return inputs.state;
  }

  public ElevatorState getDesiredState() {
    return desiredState;
  }

  public void setManualOverride(double override) {
    io.manualOverride(override);
  }

  public boolean inTolerance() {
    return inputs.aligned
        || (desiredState == ElevatorState.CORAL_L4
            && (inputs.rotPosition.getRotations() - desiredState.pos.getRotations() >= 0));
  }

  public boolean inTolerance(SuperstructureState desiredState) {
    switch (desiredState.elevatorState) {
      case CORAL_L4:
        return (inputs.rotPosition.getRotations() - desiredState.elevatorState.pos.getRotations()
                >= 0)
            || (Math.abs(
                    inputs.rotPosition.getRotations()
                        - desiredState.elevatorState.pos.getRotations())
                < ELEVATOR_TOLERANCE.getRotations()); // TODO idk why this is two conditionals
      case INTAKE_SAFE_ZONE:
        return inputs.rotPosition.getRotations()
            > desiredState.elevatorState.pos.getRotations() - 0.5;
      default:
        return Math.abs(
                inputs.rotPosition.getRotations() - desiredState.elevatorState.pos.getRotations())
            < ELEVATOR_TOLERANCE.getRotations();
    }
  }

  public void manualOverride(double percent) {
    io.manualOverride(percent);
  }

  /**
   * @param distance
   * @return a {@code Rotation2d} containing the number of rotations necessary to acheive {@code
   *     distance} units of travel, this is useful for adjusting setpoints in a more semantically
   *     meaningful way, i.e. if a setpoint is an inch off, you can add {@code
   *     distanceToElevatorRotations(Inches.of(1))} to it
   */
  public static Rotation2d distanceToElevatorRotations(Distance distance) {
    return Rotation2d.fromRotations(distance.in(Meters) * ROTATIONS_PER_METER);
  }
}
