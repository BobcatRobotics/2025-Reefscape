package frc.robot.subsystems.Superstructure.Elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Superstructure.SuperstructureState;

public class Elevator extends SubsystemBase {

  public static final Rotation2d MAX_ROTATIONS = Rotation2d.fromRotations(0); // TODO find this

  public static final Rotation2d ELEVATOR_TOLERANCE = Rotation2d.fromDegrees(2); // TODO tune

  /** the minimum height where the arm can swing freely without hitting an intake */
  public static final Rotation2d MIN_HEIGHT_INTAKE_AVOIDANCE = Rotation2d.fromRotations(0);
  /**
   * the lowest the elevator can go while the arm is upside down, for example while picking up a
   * game piece.
   */
  public static final Rotation2d MIN_HEIGHT_BOTTOM_AVOIDANCE = Rotation2d.fromRotations(0);

  /** the number of rotations the encoder spins when it is at the top */
  public static final Rotation2d ELEVATOR_MAX_ROTATIONS = new Rotation2d();

  public static final Distance ELEVATOR_MAX_HEIGHT = Inches.of(66);

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs;

  private final Alert motorDisconnectedAlert =
      new Alert("Elevator motor disconnected!", Alert.AlertType.kWarning);
  private final Alert encoderDisconnectedAlert =
      new Alert("Elevator encoder disconnected!", Alert.AlertType.kWarning);

  private SysIdRoutine routine;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    motorDisconnectedAlert.set(!inputs.motorConnected);
    encoderDisconnectedAlert.set(!inputs.encoderConnected);
  }

  public void setState(ElevatorState desiredState) {
    io.setDesiredState(desiredState);
  }

  public void runVoltage(Voltage volts) {
    io.runVoltage(volts);
  }

  public ElevatorState getState() {
    return inputs.state;
  }

  public boolean inTolerance() {
    return inputs.aligned;
  }

  public boolean inTolerance(SuperstructureState desiredState) {
    return Math.abs(
            inputs.rotPosition.getRotations() - desiredState.elevatorState.pos.getRotations())
        < ELEVATOR_TOLERANCE.getRotations();
  }
}
