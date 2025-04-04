package frc.robot.subsystems.Superstructure.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public enum ArmState {
  UPSIDE_DOWN_CCW(270), // upside down, for quick pickup once game piece intook
  RIGHT_SIDE_UP(90), // arm doesnt move from idle to handoff
  HUMAN_INTAKE(180 + 55),
  CORAL_PREP_L1(180 + 25),
  CORAL_PREP_L2(180 - 80),
  CORAL_PREP_L3(180 - 80),
  PRE_CORAL_PREP_L4(180 - 90),
  CORAL_PREP_L4(180 - 57), // TODO tune
  HANDOFF_FLIP_SAFE_ZONE(180 + 45),

  CORAL_SCORE_L1(180 + 25), // normal side
  CORAL_SCORE_L2(180 - 25),
  POST_CORAL_SCORE_L2(180 - 25),
  CORAL_SCORE_L3(180 - 25),
  CORAL_SCORE_L4(180 - 37),
  POST_CORAL_SCORE_L4(180 + 20),
  AUTO_CORAL_SCORE_L4(180 - 45),

  ALGAE_PREP_L2(180 - 0),
  ALGAE_PREP_L3(180 - 0),
  ALGAE_SCORE_L2(180 - 0),
  ALGAE_SCORE_L3(180 - 0),
  FLIPPED_ALGAE_PREP_L2(0),
  FLIPPED_ALGAE_PREP_L3(0),
  FLIPPED_ALGAE_SCORE_L2(0),
  FLIPPED_ALGAE_SCORE_L3(0),

  ALGAE_GROUND(180 + 7.5),
  ALGAE_SCORE_PROCESSOR(180 - 10),
  IDLE_ALGAE(180 - 45), // coral handoff
  NET_SCORE(50),
  NET_PREP(180 - 45),
  FLIPPED_NET_SCORE(0),
  FLIPPED_NET_PREP(25),
  CLIMB(180),
  INTAKE_SAFE_ZONE(232),
  NO_OP(0), // do nothing, maintain current pos
  UNKOWN(0); // arm isnt at a predefined state

  ArmState(double degrees) {
    this.degrees = degrees;
    this.rotations = degrees / 360;
    this.radians = Units.degreesToRadians(degrees);
    this.rot2d = Rotation2d.fromDegrees(degrees);
    // this.zone = Superstructure.getArmZone(rot2d);
  }

  public double degrees;
  // public ArmZone zone;
  public double rotations;
  public double radians;
  public Rotation2d rot2d;
}
