package frc.robot.subsystems.Superstructure.Elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.SuperstructureState;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  public static final Rotation2d ELEVATOR_TOLERANCE = Rotation2d.fromDegrees(2); // TODO tune

  /** the minimum height where the arm can swing freely without hitting an intake */
  public static final Rotation2d MIN_HEIGHT_INTAKE_AVOIDANCE = Rotation2d.fromRotations(0);
  /**
   * the lowest the elevator can go while the arm is upside down, for example while picking up a
   * game piece.
   */
  public static final Rotation2d MIN_HEIGHT_BOTTOM_AVOIDANCE = Rotation2d.fromRotations(0);

  /** the number of rotations the encoder spins when it is at the top */
  public static final Rotation2d MAX_ROTATIONS = Rotation2d.fromRadians(25.8);

  public static final Distance ELEVATOR_MAX_HEIGHT = Inches.of(66);

  /** the number of output shaft rotations per meter of elevator travel */
  public static final double ROTATIONS_PER_METER = 15.3901217;
  public static final double METERS_PER_ROTATION = 1/ROTATIONS_PER_METER;

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert = 
      new Alert("Elevator motor disconnected!", Alert.AlertType.kWarning);
  private final Alert encoderDisconnectedAlert =
      new Alert("Elevator encoder disconnected!", Alert.AlertType.kWarning);

  public Elevator(ElevatorIO io) {
    this.io = io;
    
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    motorDisconnectedAlert.set(!inputs.motorConnected);
    encoderDisconnectedAlert.set(!inputs.encoderConnected);
  }

  public void setState(ElevatorState desiredState) {
    io.setDesiredState(desiredState);
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


  public void updateMechanism2d(){

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
