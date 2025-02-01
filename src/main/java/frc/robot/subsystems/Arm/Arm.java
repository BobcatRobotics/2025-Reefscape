package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.StateMachine.SuperstructureGoal;

public class Arm {

    // see Assets\Docs\TopUpperLimit.png
    public static final Rotation2d TOP_UPPER_LIMIT = Rotation2d.fromDegrees(0);
    public static final Rotation2d TOP_LOWER_LIMIT = Rotation2d.fromDegrees(0);
    public static final Rotation2d BOTTOM_UPPER_LIMIT = Rotation2d.fromDegrees(0);
    public static final Rotation2d BOTTOM_LOWER_LIMIT = Rotation2d.fromDegrees(0);

    Rotation2d pos;

    public Arm() {
    }

    public static ArmZone getArmZone(ArmState state){
        return getArmZone(Rotation2d.fromDegrees(state.degrees));
    }
    public static ArmZone getArmZone(SuperstructureGoal goal){
        return getArmZone(goal.armState);
    }
    /**
     * see Assets\Docs\TopUpperLimit.png
     */
    @AutoLogOutput(key = "StateObserver/ArmZone")
    public static ArmZone getArmZone(Rotation2d position) {
        double deg = position.getDegrees();
        if (deg >= TOP_LOWER_LIMIT.getDegrees() && deg <= TOP_UPPER_LIMIT.getDegrees()) {
            return ArmZone.TOP_ZONE;
        } else if (deg > TOP_UPPER_LIMIT.getDegrees() && deg < BOTTOM_LOWER_LIMIT.getDegrees()) {
            return ArmZone.LEFT_INTAKE;
        } else if (deg > BOTTOM_LOWER_LIMIT.getDegrees() && deg < BOTTOM_UPPER_LIMIT.getDegrees()) {
            return ArmZone.BOTTOM_ZONE;
        } else {
            return ArmZone.RIGHT_INTAKE;
        }

    }

    public ArmZone getArmZone(){
        return getArmZone(pos);
    }
    public boolean isInIntakeZone(){
        return isInIntakeZone(getArmZone());
    }
    public static boolean isInIntakeZone(ArmZone zone){
        return zone == ArmZone.LEFT_INTAKE || zone == ArmZone.RIGHT_INTAKE;
    }

    /**
     * @return {@code false} if the the arm has to go through potential collision zones to get 
     * to the desired state from its current state, {@code true} if the current state and desired state are in the same zone. 
     * 
     * note that if the elevator is high enough, this wont matter, since it will be out of the range of the intake and bellypan
     */
    public boolean willStayInZone(ArmState desiredState) {
        return getArmZone() == getArmZone(Rotation2d.fromDegrees(desiredState.degrees));
    }

}

