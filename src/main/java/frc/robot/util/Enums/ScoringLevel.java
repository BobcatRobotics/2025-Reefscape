package frc.robot.util.Enums;

import frc.robot.subsystems.Superstructure.SuperstructureState;

public enum ScoringLevel {
  CORAL_L1(SuperstructureState.CORAL_PREP_L1, SuperstructureState.RIGHT_SIDE_UP_IDLE), // trough
  CORAL_L2(SuperstructureState.CORAL_PREP_L2, SuperstructureState.POST_CORAL_SCORE_L2),
  CORAL_L3(SuperstructureState.CORAL_PREP_L3, SuperstructureState.POST_CORAL_SCORE_L3),
  CORAL_L4(SuperstructureState.CORAL_PREP_L4, SuperstructureState.POST_CORAL_SCORE_L4),
  ALGAE_L2(SuperstructureState.ALGAE_PREP_L2, SuperstructureState.ALGAE_PREP_L2),
  ALGAE_L3(SuperstructureState.ALGAE_PREP_L3, SuperstructureState.ALGAE_PREP_L2),
  NET(SuperstructureState.NET_SCORE, SuperstructureState.RIGHT_SIDE_UP_IDLE),
  CORAL_L4_BACKWARDS(
      SuperstructureState.BACKWARDS_CORAL_PREP_L4, SuperstructureState.POST_CORAL_SCORE_L4);

  ScoringLevel(SuperstructureState prepState, SuperstructureState postState) {
    this.postState = postState;
    this.prepState = prepState;
  }

  public SuperstructureState prepState;
  public SuperstructureState postState;
}
