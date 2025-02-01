package frc.robot.subsystems.StateMachine;

import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Arm.ArmZone;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Intake.IntakeState;

public class SuperstructureGoal {
  public ElevatorState elevatorState;
  public ArmState armState;
  public IntakeState leftIntakeState;
  public IntakeState rightIntakeState;
  public ArmZone armZone;

  public SuperstructureGoal(ElevatorState elevatorState, ArmState armState, IntakeState leftIntakeState,
      IntakeState rightIntakeState) {
        this.armState = armState;
        this.elevatorState = elevatorState;
        this.leftIntakeState = leftIntakeState;
        this.rightIntakeState = rightIntakeState;
        this.armZone = Arm.getArmZone(armState);
  }

  public static SuperstructureGoal IDLE_NO_PIECE = new SuperstructureGoal(
      ElevatorState.IDLE_NO_PIECE, ArmState.IDLE_NO_PIECE, IntakeState.RETRACT, IntakeState.RETRACT);
  public static SuperstructureGoal IDLE_PIECE = new SuperstructureGoal(
      null, null, null, null);
  public static SuperstructureGoal IN_BOX = new SuperstructureGoal(
      null, null, null, null);
  public static SuperstructureGoal SCOREL1 = new SuperstructureGoal(
      null, null, null, null);
  public static SuperstructureGoal SCOREL2 = new SuperstructureGoal(
      null, null, null, null);
  public static SuperstructureGoal SCOREL3 = new SuperstructureGoal(
      null, null, null, null);
  public static SuperstructureGoal SCOREL4 = new SuperstructureGoal(
      null, null, null, null);
  public static SuperstructureGoal INTAKE_ALGAE = new SuperstructureGoal(
      null, null, null, null); // intake
  public static SuperstructureGoal PICKUP_ALGAE = new SuperstructureGoal(
      null, null, null, null); // transfer to end effector
  public static SuperstructureGoal INTAKE_CORAL_LEFT = new SuperstructureGoal(
      ElevatorState.NO_OP, ArmState.NO_OP, IntakeState.INTAKE_CORAL, IntakeState.RETRACT);
  public static SuperstructureGoal INTAKE_CORAL_RIGHT = new SuperstructureGoal(
      null, null, null, null);
  public static SuperstructureGoal INTAKE_CORAL_HUMAN = new SuperstructureGoal(
      null, null, null, null);
  public static SuperstructureGoal PICKUP_CORAL = new SuperstructureGoal(
      null, null, null, null); // transfer to end effector
  public static SuperstructureGoal CLIMB_DEPLOY = new SuperstructureGoal(
      null, null, null, null);
  public static SuperstructureGoal CLIMB_RETRACT = new SuperstructureGoal(
      null, null, null, null);

}
