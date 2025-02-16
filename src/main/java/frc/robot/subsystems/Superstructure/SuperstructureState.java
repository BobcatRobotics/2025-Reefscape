package frc.robot.subsystems.Superstructure;

import frc.robot.subsystems.Superstructure.Arm.ArmState;
import frc.robot.subsystems.Superstructure.Elevator.ElevatorState;

public enum SuperstructureState {
  UNKNOWN(ElevatorState.UNKNOWN, ArmState.UNKOWN),

  UPSIDE_DOWN_IDLE(ElevatorState.IDLE_UPSIDE_DOWN, ArmState.UPSIDE_DOWN),
  RIGHT_SIDE_UP_IDLE(ElevatorState.IDLE_RIGHT_SIDE_UP, ArmState.RIGHT_SIDE_UP),

  // Intermediate state between coral handoff and idle w/ coral, prevents collision
  // waiting for operator to confirm alignment, then bring arm down and outtake
  ELEVATOR_SAFE_ZONE(ElevatorState.INTAKE_SAFE_ZONE, ArmState.RIGHT_SIDE_UP),

  HP_INTAKE(ElevatorState.IDLE_RIGHT_SIDE_UP, ArmState.HP_INTAKE),
  CORAL_HANDOFF(ElevatorState.CORAL_HANDOFF, ArmState.UPSIDE_DOWN),

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
  ALGAE_SCORE_L3(ElevatorState.ALGAE_L2, ArmState.ALGAE_SCORE_L3),

  NET_SCORE(ElevatorState.NET_SCORE, ArmState.NET_SCORE),
  NET_PREP(ElevatorState.NET_SCORE, ArmState.NET_PREP);

  SuperstructureState(ElevatorState elevatorState, ArmState armState) {
    this.armState = armState;
    this.elevatorState = elevatorState;
  }

  public ArmState armState;
  public ElevatorState elevatorState;
}
