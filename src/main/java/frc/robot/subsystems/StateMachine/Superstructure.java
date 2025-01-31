package frc.robot.subsystems.StateMachine;

import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Elevator.Elevator;

public class Superstructure {
  private SuperstructureState currentState = SuperstructureState.IDLE;
  private Arm arm;
  private Elevator elevator;

  public Superstructure(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;
  }

  public void setState(SuperstructureState state) {
    currentState = state;
  }

  public SuperstructureState getState() {
    return currentState;
  }

  public void update() {}
}
