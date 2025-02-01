package frc.robot.subsystems.Arm;

public enum ArmState {
    IDLE_NO_PIECE(0),
    IDLE_PIECE(0),
    IN_BOX(0),
    SCOREL1(0),
    SCOREL2(0),
    SCOREL3(0),
    SCOREL4(0),
    PICKUP_ALGAE(0),
    PICKUP_CORAL(0),
    TOP_UPPER_LIMIT(0),
    TOP_LOWER_LIMIT(0),
    NO_OP(-1);

    ArmState(int degrees){
      this.degrees=degrees;
    }

    public double degrees;
  }
  
