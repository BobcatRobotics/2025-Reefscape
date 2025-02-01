package frc.robot.subsystems.StateMachine;

import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Elevator.Elevator;

public class Superstructure {
  private SuperstructureGoal currentState = SuperstructureGoal.IN_BOX;
  private SuperstructureGoal desiredState = SuperstructureGoal.IN_BOX;
  private Arm arm;
  private Elevator elevator;


  public Superstructure(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;
  }

  public void setState(SuperstructureGoal desiredState) {
    if(transitionValid(desiredState)) {
      currentState = desiredState;
    }else {
      //TODO create updated state
    }

  }

  public SuperstructureGoal getState() {
    return currentState;
  }

  public boolean transitionValid(SuperstructureGoal desiredState) {
    switch (desiredState.armState){
      case PICKUP_ALGAE:
        if(elevator.)
    } 
  }


}
