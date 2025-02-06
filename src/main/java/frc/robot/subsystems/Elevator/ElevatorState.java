package frc.robot.subsystems.Elevator;

public enum ElevatorState {
  IDLE_NO_PIECE(0), // upside down, for quick pickup once game piece intook
  IDLE_CORAL(0), // right side up for quick transition to scoring zone
  CORAL_HANDOFF(0), // picking up coral with the end effector
  CORAL_L1(0), // elevator pos doesnt matter for score or prep
  CORAL_L2(0),
  CORAL_L3(0),
  CORAL_L4(0),
  INTAKE_SAFE_ZONE(0), // safe height for flipping up the arm
  ALGAE_L2(0),
  ALGAE_L3(0),
  ALGAE_GROUND(0), // pickup algae from ground
  ALGAE_SCORE_PROCESSOR(0), // pickup algae from processor
  IDLE_ALGAE(0); // hold a algae

  ElevatorState(double heightMeters) {
    this.heightMeters = heightMeters;
  }

  public double heightMeters;
}
