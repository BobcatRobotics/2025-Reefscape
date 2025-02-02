package frc.robot.subsystems.Arm;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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
    this.rotations = degrees/360;
    this.zone = zone;
    this.radians = Units.degreesToRadians(degrees);
    this.rot2d = Rotation2d.fromDegrees(degrees);
  }

  public double degrees;
  public ArmZone zone;
  public double rotations;
  public double radians;
  public Rotation2d rot2d;
}
