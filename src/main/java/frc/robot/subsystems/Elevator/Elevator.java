package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** the minimum height where the arm can swing freely without hitting an intake */
  public static final Distance MIN_HEIGHT_INTAKE_AVOIDANCE = Meters.of(0);
  /**
   * the lowest the elevator can go while the arm is upside down, for example while picking up a
   * game piece.
   */
  public static final Distance MIN_HEIGHT_BOTTOM_AVOIDANCE = Meters.of(0);

  /** the highest the elevator can go from the bottom */
  public static final Distance ELEVATOR_MAX_HEIGHT = Meters.of(0);

  /** the number of rotations the encoder spins when it is at the top */
  public static final Rotation2d ELEVATOR_MAX_ROTATIONS = new Rotation2d();

  private Distance height;
  private ElevatorState currentState = ElevatorState.IDLE_NO_PIECE;

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    io.setDesiredState(currentState);
  }

  public Distance getPos() {
    return inputs.position;
  }

  public void setState(ElevatorState desiredState) {
    currentState = desiredState;
  }

  public ElevatorState getState() {
    return currentState;
  }

  public Distance getHeight() {
    return height;
  }

  public double getHeightMeters() {
    return getHeight().in(Meters);
  }
}
