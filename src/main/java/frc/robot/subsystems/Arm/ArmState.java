package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public enum ArmState {
  IDLE_NO_PIECE(-90), // upside down, for quick pickup once game piece intook
  IDLE_CORAL(90), // arm doesnt move from idle to handoff
  CORAL_SCORE_L1(0),
  CORAL_SCORE_L2(0),
  CORAL_SCORE_L3(0),
  CORAL_SCORE_L4(0),
  CORAL_PREP_L1(0),
  CORAL_PREP_L2(0),
  CORAL_PREP_L3(0),
  CORAL_PREP_L4(0),
  ALGAE_PREP_L2(0),
  ALGAE_PREP_L3(0),
  ALGAE_SCORE_L2(0),
  ALGAE_SCORE_L3(0),
  ALGAE_GROUND(0),
  ALGAE_SCORE_PROCESSOR(0),
  IDLE_ALGAE(0); // coral handoff

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
