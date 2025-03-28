package frc.robot.subsystems.Superstructure.Elevator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public enum ElevatorState {
  IDLE_UPSIDE_DOWN(
      Rotation2d.fromRotations(1.3)), // upside down, for quick pickup once game piece intook
  IDLE_RIGHT_SIDE_UP(
      Rotation2d.fromRotations(0)), // right side up for quick transition to scoring zone
  HANDOFF_PREP(Rotation2d.fromRotations(1.15)),
  CORAL_HANDOFF(
      Rotation2d.fromRotations(0.95)), // picking up coral with the end effector //OLD 1.015
  CORAL_L1(Rotation2d.fromRotations(1.35)),

  CORAL_PREP_L2(Rotation2d.fromRotations(0.65)),
  CORAL_SCORE_L2(Rotation2d.fromRotations(0.65)),

  POST_CORAL_L2(Rotation2d.fromRotations(0)),
  CORAL_L3(Rotation2d.fromRotations(1.63)),
  POST_CORAL_L3(Rotation2d.fromRotations(0.4)),
  CORAL_L4(Rotation2d.fromRotations(3)),
  POST_CORAL_L4(Rotation2d.fromRotations(3)),
  INTAKE_SAFE_ZONE(Rotation2d.fromRotations(2.5)), // safe height for flipping up the arm
  ALGAE_L2(Rotation2d.fromRotations(1.45)),
  ALGAE_L3(Rotation2d.fromRotations(2.2)),
  ALGAE_GROUND(Rotation2d.fromRotations(0)), // pickup algae from ground
  ALGAE_SCORE_PROCESSOR(Rotation2d.fromRotations(0)), // pickup algae from processor
  IDLE_ALGAE(Rotation2d.fromRotations(0)), // hold a algae
  NET_SCORE(Rotation2d.fromRotations(4.1)),
  NET_PREP(Rotation2d.fromRotations(4.1)),
  UNKNOWN(Rotation2d.fromRotations(0)),
  CLIMB(Rotation2d.fromRotations(0)),
  HUMAN_INTAKE(Rotation2d.fromRotations(2.8)),
  POST_HUMAN_INTAKE(Rotation2d.fromRotations(2.3)); // elevator isnt at a predefined state

  ElevatorState(Rotation2d pos) {
    this.pos = pos;
    heightMeters = pos.getRotations() * Elevator.METERS_PER_ROTATION;
    height = Meters.of(heightMeters);
  }

  public Rotation2d pos;
  public Distance height;
  public double heightMeters;
}
