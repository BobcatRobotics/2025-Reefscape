package frc.robot.subsystems.StateMachine;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Arm.ArmZone;
import frc.robot.subsystems.CoralIntake.IntakeState;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class StateObserver {
  private static StateObserver instance;

  public ArmState armstate = ArmState.IDLE_NO_PIECE;
  public ArmZone armZone = ArmZone.BOTTOM_ZONE;
  public Rotation2d armPos = new Rotation2d();

  public ElevatorState elevatorState = ElevatorState.IDLE_NO_PIECE;
  public Distance elevatorHeight = Meters.of(0);

  public IntakeState coralIntakeState = IntakeState.RETRACT;
  public IntakeState algaeIntakeState = IntakeState.RETRACT;

  private StateObserver() {}

  public void updateArm(ArmState armState, Rotation2d armPos) {
    this.armstate = armState;
    this.armPos = armPos;
    armZone = Arm.getArmZone(armPos);
  }

  public void updateElevator(ElevatorState elevatorState, Distance elevatorHeight) {
    this.elevatorHeight = elevatorHeight;
    this.elevatorState = elevatorState;
  }

  public static StateObserver getInstance() {
    if (instance == null) {
      instance = new StateObserver();
    }
    return instance;
  }

  /**
   * @return {@code true} if the elevator is at or above the min position where the arm can swing
   *     freely without hitting the intake
   */
  public boolean elevatorAboveIntakeMinimum() {
    return elevatorHeight.baseUnitMagnitude()
        >= Elevator.MIN_HEIGHT_INTAKE_AVOIDANCE.baseUnitMagnitude();
  }

  /**
   * @return {@code true} if the elevator is at or above the min position where the arm can swing
   *     freely without hitting the floor
   *     <p>WARNING: the elevator may still be able to hit the intake at this height
   */
  public boolean elevatorAboveFloorMinimum() {
    return elevatorHeight.baseUnitMagnitude()
        >= Elevator.MIN_HEIGHT_BOTTOM_AVOIDANCE.baseUnitMagnitude();
  }

  /**
   * @param zone
   * @return {@code true} if the given armzone is the coral or algae zone
   */
  public boolean isInIntakeZone() {
    return armZone == ArmZone.CORAL_INTAKE || armZone == ArmZone.ALGAE_INTAKE;
  }

  /**
   * @param zone
   * @return {@code true} if the given armzone is the coral or algae zone
   */
  public static boolean isInIntakeZone(ArmZone zone) {
    return zone == ArmZone.CORAL_INTAKE || zone == ArmZone.ALGAE_INTAKE;
  }

}
