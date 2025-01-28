package frc.robot.subsystems.StateObserver;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.Constants.ArmConstants;
import frc.robot.Constants.Constants.IntakeConstants;

public class SuperstructureState {
    public Distance elevatorHeight;
    public Angle armAngle;
    public AngularVelocity intakeSpeed;
    
    public SuperstructureState(Distance elevatorHeight, Angle armAngle, AngularVelocity intakeSpeed){
        this.elevatorHeight = elevatorHeight;
        this.armAngle = armAngle;
        this.intakeSpeed = intakeSpeed;
    }

    public SuperstructureState(){
        this.elevatorHeight = Meters.zero();
        this.armAngle = Radians.zero();
        this.intakeSpeed = RadiansPerSecond.zero();
    }

    /**
     * in meters
     * @return
     */
    public Translation2d endEffectorPosition(){
        //component of the arm pointing in the x direction
        double x = 
            Math.cos(armAngle.in(Radians)) 
            * (ArmConstants.armLength.in(Meters) + IntakeConstants.distanceToCenter.in(Meters)) 
            ;
        
        //component of the arm and end effector pointing in the y direction plus elevator height
        double y =
            Math.sin(armAngle.in(Radians))
            * (ArmConstants.armLength.in(Meters) + IntakeConstants.distanceToCenter.in(Meters)) 
            + elevatorHeight.in(Meters);

        return new Translation2d(x, y);
    }


    public void updateElevatorHeight(Distance newHeight){
        this.elevatorHeight = newHeight;
    }
    public void updateArmAngle(Angle newAngle){
        this.armAngle = newAngle;
    }
    public void updateIntakeSpeed(AngularVelocity newSpeed){
        this.intakeSpeed = newSpeed;
    }
    public void update(Distance newHeight, Angle newAngle, AngularVelocity newIntakeSpeed){
        this.elevatorHeight = newHeight;
        this.armAngle = newAngle;
        this.intakeSpeed = newIntakeSpeed;
    }


    public boolean equals(SuperstructureState other){
        return this.elevatorHeight == other.elevatorHeight 
        && this.armAngle == other.armAngle 
        && this.intakeSpeed == other.intakeSpeed;
    }

    //TODO add other comparison methods
}
