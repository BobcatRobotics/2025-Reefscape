package frc.robot.subsystems.StateMachine;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Arm.ArmZone;
import frc.robot.subsystems.CoralIntake.IntakeState;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class StateObserver {
    public ArmState armstate;
    public ArmZone armZone;
    public Rotation2d armPos;

    public ElevatorState elevatorState;
    public Distance elevatorHeight;

    public IntakeState coralIntakeState;
    public IntakeState algaeIntakeState;

    public StateObserver(Superstructure superstructure) {
    }

    public void updateArm(ArmState armState, Rotation2d armPos) {
        this.armstate = armState;
        this.armPos = armPos;
        armZone = Arm.getArmZone(armPos);
    }

    public void updateElevator(ElevatorState elevatorState, Distance elevatorHeight){
        this.elevatorHeight = elevatorHeight;
        this.elevatorState = elevatorState;
    }

    /**
     * @return {@code true} if the elevator is at or above the min position where
     * the arm can swing freely without hitting the intake
     */
    public boolean elevatorAboveIntakeMinimum(){
        return elevatorHeight.baseUnitMagnitude() >= Elevator.MIN_HEIGHT_INTAKE_AVOIDANCE.baseUnitMagnitude();
    }

    /**
     * @return {@code true} if the elevator is at or above the min position where
     * the arm can swing freely without hitting the floor
     * 
     * WARNING: the elevator may still be able to hit the intake at this height
     */
    public boolean elevatorAboveFloorMinimum(){
        return elevatorHeight.baseUnitMagnitude() >= Elevator.MIN_HEIGHT_BOTTOM_AVOIDANCE.baseUnitMagnitude();
    }




    public static boolean isInIntakeZone(ArmZone zone) {
        return zone == ArmZone.CORAL_INTAKE || zone == ArmZone.ALGAE_INTAKE;
    }

}
