package frc.robot.util.Enums;

import frc.robot.subsystems.Superstructure.SuperstructureState;

public enum ScoringLevel {
  CORAL_L1(SuperstructureState.RIGHT_SIDE_UP_IDLE), // trough
  CORAL_L2(SuperstructureState.POST_CORAL_SCORE_L2),
  CORAL_L3(SuperstructureState.POST_CORAL_SCORE_L3),
  CORAL_L4(SuperstructureState.POST_CORAL_SCORE_L4),
  ALGAE_L2(SuperstructureState.ALGAE_LIFT_L2),
  ALGAE_L3(SuperstructureState.ALGAE_LIFT_L3),
  NET(SuperstructureState.RIGHT_SIDE_UP_IDLE);

  ScoringLevel(SuperstructureState postState) {
    this.postState = postState;
  }

  public SuperstructureState postState;
}
