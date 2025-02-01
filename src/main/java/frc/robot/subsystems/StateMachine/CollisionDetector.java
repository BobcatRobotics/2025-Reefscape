package frc.robot.subsystems.StateMachine;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intake.Intake;

public class CollisionDetector {
    // the minimum height the elevator has to be at for the arm to
    // swing fully down or up without collision while the intake is deployed
    public static final Distance ARM_SWING_MIN_HEIGHT_INTAKE_DEPLOYED = Meters.of(0); // TODO find this

    // the minimum height the elevator has to be at for the arm to
    // swing fully down pr up without collision while the intake is retracted
    public static final Distance ARM_SWING_MIN_HEIGHT_INTAKE_RETRACTED = Meters.of(0); // TODO find this

    private Arm arm;
    private Elevator elevator;
    private Intake intake;
    private SuperstructureGoal desiredState;
    private SuperstructureGoal currentState;

    public CollisionDetector(Elevator elevator, Arm arm, Intake intake) {
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
    }

    /**
     * 
     * @param goal    desired state to go to
     * @param current current state of the robot
     * @return a safe sequence of SuperstructureGoals to travel to that avoids
     *         collisions
     */
    public SuperstructureGoal[] getSafeState(SuperstructureGoal goal, SuperstructureGoal current) {
        desiredState = goal;
        currentState = current;
    }

    public boolean isTransitionSafe(SuperstructureGoal goal) {
        
        // make sure state is valid
        // will the arm hit the intake or floor of the robot?
        if(!Elevator.armWontCollideWithBottom(goal.armZone, goal.elevatorState)){
            return false;
        };

        //if the arm swings down before the elevator raises, will it hit?
        if() 

        

        

    }

}
