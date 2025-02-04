package frc.robot.subsystems.StateMachine;

import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Arm.ArmZone;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.CoralIntake.IntakeState;
import frc.robot.subsystems.Elevator.ElevatorState;

public class SuperstructureState {
  public ElevatorState elevatorState;
  public ArmState armState;
  public IntakeState coralIntakeState;
  public IntakeState algaeIntakeState;
  public ArmZone armZone;
  public ClimberState climberState;

  public SuperstructureState(
      ElevatorState elevatorState,
      ArmState armState,
      IntakeState coralIntakeState,
      IntakeState algaeIntakeState,
      ClimberState climberState) {
    this.armState = armState;
    this.elevatorState = elevatorState;
    this.coralIntakeState = coralIntakeState;
    this.algaeIntakeState = algaeIntakeState;
    this.climberState = climberState;
    this.armZone = Arm.getArmZone(armState);
  }

  //TODO im an idiot this should be an enum

  public static SuperstructureState IDLE_NO_PIECE =
      new SuperstructureState(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureState IDLE_PIECE =
      new SuperstructureState(
          ElevatorState.IDLE_PIECE,
          ArmState.IDLE_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureState IN_BOX = // TODO update all of these
      new SuperstructureState(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureState SCOREL1 =
      new SuperstructureState(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureState SCOREL3 =
      new SuperstructureState(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureState SCOREL4 =
      new SuperstructureState(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureState INTAKE_ALGAE =
      new SuperstructureState(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW); // intake

  public static SuperstructureState PICKUP_ALGAE =
      new SuperstructureState(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureState INTAKE_CORAL =
      new SuperstructureState(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureState INTAKE_CORAL_HUMAN =
      new SuperstructureState(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureState PICKUP_CORAL =
      new SuperstructureState(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW); // transfer to end effector

  public static SuperstructureState CLIMB_DEPLOY =
      new SuperstructureState(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureState CLIMB_RETRACT =
      new SuperstructureState(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);
}
