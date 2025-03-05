package frc.robot.util.Enums;

import frc.robot.subsystems.Superstructure.SuperstructureState;

public enum IdleType {
  UPRIGHT(SuperstructureState.RIGHT_SIDE_UP_IDLE),
  UPSIDE_DOWN(SuperstructureState.UPSIDE_DOWN_IDLE);

  IdleType(SuperstructureState state) {
    this.state = state;
  }

  public SuperstructureState state;
}
