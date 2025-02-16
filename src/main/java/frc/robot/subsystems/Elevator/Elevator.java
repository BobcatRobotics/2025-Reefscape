package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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


  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
  }


  public void setState(ElevatorState desiredState) {
    io.setDesiredState(desiredState);
  }

  public ElevatorState getState() {
    return inputs.state;
  }

  public boolean inTolerance(){
    return inputs.aligned;
  }
  public boolean inTolerance(SuperstructureState desiredState){
    return Math.abs(
      inputs.rotPosition.getRotations() - desiredState.elevatorState.pos.getRotations()) 
      < ELEVATOR_TOLERANCE.getRotations();

  }

}
