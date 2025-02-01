package frc.robot.subsystems.StateMachine;

import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Arm.ArmZone;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.CoralIntake.IntakeState;
import frc.robot.subsystems.Elevator.ElevatorState;

public class SuperstructureGoal {
  public ElevatorState elevatorState;
  public ArmState armState;
  public IntakeState coralIntakeState;
  public IntakeState algaeIntakeState;
  public ArmZone armZone;
  public ClimberState climberState;

  public SuperstructureGoal(
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

  public static SuperstructureGoal IDLE_NO_PIECE =
      new SuperstructureGoal(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureGoal IDLE_PIECE =
      new SuperstructureGoal(
          ElevatorState.IDLE_PIECE,
          ArmState.IDLE_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureGoal IN_BOX =
      new SuperstructureGoal(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureGoal SCOREL1 =
      new SuperstructureGoal(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureGoal SCOREL3 =
      new SuperstructureGoal(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureGoal SCOREL4 =
      new SuperstructureGoal(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureGoal INTAKE_ALGAE =
      new SuperstructureGoal(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW); // intake

  public static SuperstructureGoal PICKUP_ALGAE =
      new SuperstructureGoal(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureGoal INTAKE_CORAL =
      new SuperstructureGoal(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureGoal INTAKE_CORAL_HUMAN =
      new SuperstructureGoal(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureGoal PICKUP_CORAL =
      new SuperstructureGoal(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW); // transfer to end effector

  public static SuperstructureGoal CLIMB_DEPLOY =
      new SuperstructureGoal(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);

  public static SuperstructureGoal CLIMB_RETRACT =
      new SuperstructureGoal(
          ElevatorState.IDLE_NO_PIECE,
          ArmState.IDLE_NO_PIECE,
          IntakeState.RETRACT,
          IntakeState.RETRACT,
          ClimberState.STOW);
}
