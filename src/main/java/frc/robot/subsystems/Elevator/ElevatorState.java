package frc.robot.subsystems.Elevator;
import edu.wpi.first.math.geometry.Rotation2d;

public enum ElevatorState {
  IDLE_UPSIDE_DOWN(Rotation2d.fromRotations(0)), // upside down, for quick pickup once game piece intook
  IDLE_RIGHT_SIDE_UP(Rotation2d.fromRotations(0)), // right side up for quick transition to scoring zone
  CORAL_HANDOFF(Rotation2d.fromRotations(0)), // picking up coral with the end effector
  CORAL_L1(Rotation2d.fromRotations(0)), // elevator pos doesnt matter for score or prep
  CORAL_L2(Rotation2d.fromRotations(0)),
  CORAL_L3(Rotation2d.fromRotations(0)),
  CORAL_L4(Rotation2d.fromRotations(0)),
  INTAKE_SAFE_ZONE(Rotation2d.fromRotations(0)), // safe height for flipping up the arm
  ALGAE_L2(Rotation2d.fromRotations(0)),
  ALGAE_L3(Rotation2d.fromRotations(0)),
  ALGAE_GROUND(Rotation2d.fromRotations(0)), // pickup algae from ground
  ALGAE_SCORE_PROCESSOR(Rotation2d.fromRotations(0)), // pickup algae from processor
  IDLE_ALGAE(Rotation2d.fromRotations(0)), // hold a algae
  NET_SCORE(Rotation2d.fromRotations(0)),
  UNKNOWN(Rotation2d.fromRotations(0)); // elevator isnt at a predefined state

  ElevatorState(Rotation2d pos) {
    this.pos = pos;
    
  }

  public Rotation2d pos;
}
