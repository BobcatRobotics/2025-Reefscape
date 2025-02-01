package frc.robot.subsystems.Elevator;

public enum ElevatorState {
  IDLE_NO_PIECE(0), // upside down, for quick pickup once game piece intook
  IDLE_PIECE(
      0), // right side up, for quick transition to scoring zone, and so that we dont collide with
  // the reef
  IN_BOX(0), // starting config
  SCOREL1(0),
  SCOREL2(0),
  SCOREL3(0),
  SCOREL4(0),
  PICKUP_ALGAE(0), // algae handoff
  PICKUP_CORAL(0), // coral handoff
  NO_OP(0); // maintain current position

  ElevatorState(double heightMeters) {
    this.heightMeters = heightMeters;
  }

  public double heightMeters;
}
