package frc.robot.subsystems.Arm;

public enum ArmState {
  IDLE_NO_PIECE(0, ArmZone.BOTTOM_ZONE),
  IDLE_PIECE(0, ArmZone.TOP_ZONE),
  IN_BOX(0, ArmZone.TOP_ZONE),
  SCOREL1(0, ArmZone.TOP_ZONE),
  SCOREL2(0, ArmZone.TOP_ZONE),
  SCOREL3(0, ArmZone.TOP_ZONE),
  SCOREL4(0, ArmZone.TOP_ZONE),
  PICKUP_ALGAE(0, ArmZone.BOTTOM_ZONE),
  PICKUP_CORAL(0, ArmZone.BOTTOM_ZONE),
  NO_OP(-1, ArmZone.BOTTOM_ZONE);

  ArmState(int degrees, ArmZone zone) {
    this.degrees = degrees;
  }

  public double degrees;
  public double zone;
}
