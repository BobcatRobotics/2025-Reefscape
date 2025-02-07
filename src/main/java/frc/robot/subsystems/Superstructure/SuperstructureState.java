package frc.robot.subsystems.Superstructure;

import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator.ElevatorState;

public enum SuperstructureState {
  UNKNOWN(ElevatorState.UNKNOWN, ArmState.UNKOWN),
  IDLE_NO_PIECE(ElevatorState.IDLE_NO_PIECE, ArmState.IDLE_NO_PIECE),
  CORAL_HANDOFF(ElevatorState.CORAL_HANDOFF, ArmState.IDLE_NO_PIECE),
  // Intermediate state between coral handoff and idle w/ coral, prevents collision
  ELEVATOR_SAFE_ZONE(ElevatorState.INTAKE_SAFE_ZONE, ArmState.IDLE_NO_PIECE),
  IDLE_CORAL(ElevatorState.IDLE_CORAL, ArmState.IDLE_CORAL),
  // waiting for operator to confirm alignment, then bring arm down and outtake
  CORAL_PREP_L1(ElevatorState.CORAL_L1, ArmState.CORAL_PREP_L1),
  CORAL_PREP_L2(ElevatorState.CORAL_L2, ArmState.CORAL_PREP_L2),
  CORAL_PREP_L3(ElevatorState.CORAL_L3, ArmState.CORAL_PREP_L3),
  CORAL_PREP_L4(ElevatorState.CORAL_L4, ArmState.CORAL_PREP_L4),
  CORAL_SCORE_L1(ElevatorState.CORAL_L1, ArmState.CORAL_SCORE_L1),
  CORAL_SCORE_L2(ElevatorState.CORAL_L2, ArmState.CORAL_SCORE_L2),
  CORAL_SCORE_L3(ElevatorState.CORAL_L3, ArmState.CORAL_SCORE_L3),
  CORAL_SCORE_L4(ElevatorState.CORAL_L4, ArmState.CORAL_SCORE_L4),
  IDLE_ALGAE(ElevatorState.IDLE_ALGAE, ArmState.IDLE_ALGAE),
  ALGAE_SCORE_PROCESSOR(ElevatorState.ALGAE_SCORE_PROCESSOR, ArmState.ALGAE_SCORE_PROCESSOR),
  INTAKE_ALGAE_GROUND(ElevatorState.ALGAE_GROUND, ArmState.ALGAE_GROUND),
  ALGAE_PREP_L2(ElevatorState.ALGAE_L2, ArmState.ALGAE_PREP_L2),
  ALGAE_PREP_L3(
      ElevatorState.ALGAE_L2,
      ArmState
          .ALGAE_PREP_L3), // TODO is this neccesary? should we have a algae retract state instead?
  ALGAE_SCORE_L2(ElevatorState.ALGAE_L2, ArmState.ALGAE_SCORE_L2),
  ALGAE_SCORE_L3(ElevatorState.ALGAE_L2, ArmState.ALGAE_SCORE_L3);

  SuperstructureState(ElevatorState elevatorState, ArmState armState) {
    this.armState = armState;
    this.elevatorState = elevatorState;
  }

  public ArmState armState;
  public ElevatorState elevatorState;
}
