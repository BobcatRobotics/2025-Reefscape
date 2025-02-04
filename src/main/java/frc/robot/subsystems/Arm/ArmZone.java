package frc.robot.subsystems.Arm;

public enum ArmZone {
  TOP_ZONE, // centered at 90 deg
  BOTTOM_ZONE, // centered at 270 deg TODO it should be impossible to hit the intake while the arm
  // is in this zone
  CORAL_INTAKE, // centered at 180 deg
  ALGAE_INTAKE // centered at 0 deg
}
