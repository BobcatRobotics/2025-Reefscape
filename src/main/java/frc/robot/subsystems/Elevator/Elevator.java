package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Arm.ArmZone;

public class Elevator {

    private Distance height;

    // the minimum height where the arm can swing freely without hitting an intake
    public static final Distance MIN_HEIGHT_INTAKE_AVOIDANCE = Meters.of(0);

    // the lowest the elevator can go while the arm is upside down,
    // for example while picking up a game piece.
    public static final Distance MIN_HEIGHT_BOTTOM_AVOIDANCE = Meters.of(0);

    public Elevator() {

    }

    public Distance getHeight() {
        return height;
    }

    public double getHeightMeters() {
        return getHeight().in(Meters);
    }

    /**
     * @return {@code true} if the elevator is high enough where the arm can't
     * collide with the intake
     */
    public static boolean armWontCollideWithBottom(ArmZone zone, ElevatorState elevatorState) {
        if (Arm.isInIntakeZone(zone)) {
            return elevatorState.heightMeters > MIN_HEIGHT_INTAKE_AVOIDANCE.in(Meters);
        } else if (zone == ArmZone.BOTTOM_ZONE) {
            return elevatorState.heightMeters > MIN_HEIGHT_BOTTOM_AVOIDANCE.in(Meters);
        } else {
            return true;
        }
    }

}
