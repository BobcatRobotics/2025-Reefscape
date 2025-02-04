package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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
  CLIMB(0),
  HUMAN_INTAKE(0),
  NO_OP(-1);


  ArmState(int degrees) {
    this.degrees = degrees;
    this.rotations = degrees / 360;
    this.radians = Units.degreesToRadians(degrees);
    this.rot2d = Rotation2d.fromDegrees(degrees);
    this.zone = Arm.getArmZone(rot2d);
  }

  public double degrees;
  public ArmZone zone;
  public double rotations;
  public double radians;
  public Rotation2d rot2d;
}
