package frc.robot.subsystems.StateMachine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Arm.ArmZone;
import frc.robot.subsystems.CoralIntake.IntakeState;
import frc.robot.subsystems.Elevator.ElevatorState;

public class StateObserver {
    public ArmState armstate;
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
    }

    public static boolean isInIntakeZone(ArmZone zone) {
        return zone == ArmZone.LEFT_INTAKE || zone == ArmZone.RIGHT_INTAKE;
    }

}
