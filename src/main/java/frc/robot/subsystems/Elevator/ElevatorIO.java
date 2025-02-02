package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs{
        Distance position;
        ElevatorState state;
    }

    public default void updateInputs(){}
    
    public default void setDesiredState(){}
}
