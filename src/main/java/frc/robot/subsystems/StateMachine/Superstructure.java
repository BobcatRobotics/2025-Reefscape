package frc.robot.subsystems.StateMachine;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntake;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.Elevator.Elevator;

public class Superstructure {
  private SuperstructureState currentState = SuperstructureState.IN_BOX;
  private Arm arm;
  private Elevator elevator;
  private CoralIntake coralIntake;
  private AlgaeIntake algaeIntake;
  Alert stateAlert = new Alert("Attempted to set drivetrain to invalid state!", AlertType.kError);

  public Superstructure(
      Arm arm, Elevator elevator, AlgaeIntake algaeIntake, CoralIntake coralIntake) {
    this.arm = arm;
    this.elevator = elevator;
    this.coralIntake = coralIntake;
    this.algaeIntake = algaeIntake;
    stateAlert.set(false);
  }

  /**
   * @param desiredState the state we want to set the robot to
   * @return {@code true} if the state was set, {@code false} if the state was invalid
   */
  public boolean setState(SuperstructureState desiredState) {
    // make sure state is valid
    // will the arm hit the intake or floor of the robot in its final state?
    if (Elevator.checkForArmCollision(desiredState.armZone, desiredState.elevatorState)) {
      stateAlert.set(true);
      return false; // dont set the arm to the invalid state
    }
    ;

    currentState = desiredState;

    elevator.setState(desiredState.elevatorState);
    arm.setState(desiredState.armState);
    algaeIntake.setState(desiredState.algaeIntakeState);
    coralIntake.setState(desiredState.coralIntakeState);
    return true;
  }

  public SuperstructureState getState() {
    return currentState;
  }

  /**
   * @param goal the desired state
   * @return whether or not the elevator will have to move before the arm swings fully down
   */
  public boolean isTransitionSafe(SuperstructureState goal) {
    // make sure state is valid
    // will the arm hit the intake or floor of the robot in its final state?
    if (Elevator.checkForArmCollision(goal.armZone, goal.elevatorState)) {
      return false;
    }
    ;

    // if the arm swings down before the elevator raises, will it hit?
    if (Elevator.checkForArmCollision(goal.armZone, currentState.elevatorState)) {
      return false;
    }
    ;
    return true;
  }
}
