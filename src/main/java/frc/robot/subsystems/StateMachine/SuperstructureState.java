package frc.robot.subsystems.StateMachine;

import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.CoralIntake.IntakeState;
import frc.robot.subsystems.Elevator.ElevatorState;

// public class SuperstructureState {
//   public ElevatorState elevatorState;
//   public ArmState armState;
//   public IntakeState coralIntakeState;
//   public IntakeState algaeIntakeState;
//   public ArmZone armZone;
//   public ClimberState climberState;

//   public SuperstructureState(
//       ElevatorState elevatorState,
//       ArmState armState,
//       IntakeState coralIntakeState,
//       IntakeState algaeIntakeState,
//       ClimberState climberState) {
//     this.armState = armState;
//     this.elevatorState = elevatorState;
//     this.coralIntakeState = coralIntakeState;
//     this.algaeIntakeState = algaeIntakeState;
//     this.climberState = climberState;
//     this.armZone = Arm.getArmZone(armState);
//   }

//   //TODO im an idiot this should be an enum

//   public static SuperstructureState IDLE_NO_PIECE =
//       new SuperstructureState(
//           ElevatorState.IDLE_NO_PIECE,
//           ArmState.IDLE_NO_PIECE,
//           IntakeState.RETRACT,
//           IntakeState.RETRACT,
//           ClimberState.STOW);

//   public static SuperstructureState IDLE_PIECE =
//       new SuperstructureState(
//           ElevatorState.IDLE_PIECE,
//           ArmState.IDLE_PIECE,
//           IntakeState.RETRACT,
//           IntakeState.RETRACT,
//           ClimberState.STOW);

//   public static SuperstructureState IN_BOX = // TODO update all of these
//       new SuperstructureState(
//           ElevatorState.IDLE_NO_PIECE,
//           ArmState.IDLE_NO_PIECE,
//           IntakeState.RETRACT,
//           IntakeState.RETRACT,
//           ClimberState.STOW);

//   public static SuperstructureState SCOREL1 =
//       new SuperstructureState(
//           ElevatorState.IDLE_NO_PIECE,
//           ArmState.IDLE_NO_PIECE,
//           IntakeState.RETRACT,
//           IntakeState.RETRACT,
//           ClimberState.STOW);

//   public static SuperstructureState SCOREL3 =
//       new SuperstructureState(
//           ElevatorState.IDLE_NO_PIECE,
//           ArmState.IDLE_NO_PIECE,
//           IntakeState.RETRACT,
//           IntakeState.RETRACT,
//           ClimberState.STOW);

//   public static SuperstructureState SCOREL4 =
//       new SuperstructureState(
//           ElevatorState.IDLE_NO_PIECE,
//           ArmState.IDLE_NO_PIECE,
//           IntakeState.RETRACT,
//           IntakeState.RETRACT,
//           ClimberState.STOW);

//   public static SuperstructureState INTAKE_ALGAE =
//       new SuperstructureState(
//           ElevatorState.IDLE_NO_PIECE,
//           ArmState.IDLE_NO_PIECE,
//           IntakeState.RETRACT,
//           IntakeState.RETRACT,
//           ClimberState.STOW); // intake

public enum SuperstructureState {
  INTAKE_ALGAE(
      ElevatorState.IDLE_NO_PIECE,
      ArmState.IDLE_NO_PIECE,
      IntakeState.RETRACT,
      IntakeState.RETRACT),
  PICKUP_ALGAE(
      ElevatorState.PICKUP_ALGAE, ArmState.PICKUP_ALGAE, IntakeState.RETRACT, IntakeState.RETRACT),
  CORAL_HANDOFF(
      ElevatorState.PICKUP_CORAL, ArmState.PICKUP_CORAL, IntakeState.RETRACT, IntakeState.RETRACT),
  INTAKE_CORAL(
      ElevatorState.IDLE_NO_PIECE, ArmState.IDLE_NO_PIECE, IntakeState.DEPLOY, IntakeState.RETRACT),
  INTAKE_CORAL_HUMAN(
      ElevatorState.HUMAN_INTAKE, ArmState.PICKUP_CORAL, IntakeState.RETRACT, IntakeState.RETRACT),
  PICKUP_CORAL(
      ElevatorState.PICKUP_CORAL, ArmState.PICKUP_CORAL, IntakeState.RETRACT, IntakeState.RETRACT),
  CLIMB_DEPLOY(ElevatorState.CLIMB, ArmState.CLIMB, IntakeState.RETRACT, IntakeState.RETRACT),
  CLIMB_RETRACT(ElevatorState.CLIMB, ArmState.CLIMB, IntakeState.RETRACT, IntakeState.RETRACT);

  SuperstructureState(
      ElevatorState elevatorState,
      ArmState armState,
      IntakeState coralIntakeState,
      IntakeState algaeIntakeState) {
    this.armState = armState;
    this.elevatorState = elevatorState;
    this.coralIntakeState = coralIntakeState;
    this.algaeIntakeState = algaeIntakeState;
  }

  public ArmState armState;
  public ElevatorState elevatorState;
  public IntakeState coralIntakeState;
  public IntakeState algaeIntakeState;
}
