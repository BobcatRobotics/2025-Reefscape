package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.Constants;
import au.grapplerobotics.LaserCan;

public class EndEffector { //change to EndEffector
  private boolean deployed;
  private LaserCan lc;

  /** Distance between minimum elevator position and top of the coral intake */
  public static final Distance CORAL_INTAKE_HEIGHT = Meters.of(0);

  private TalonFX topMotor;
  private DutyCycleOut request;

  public EndEffector() { //change name to endEffector. Coral intake is the floor intake
    topMotor = new TalonFX(Constants.IntakeConstants.topMotorID);
    request = new DutyCycleOut(Constants.IntakeConstants.motorCurrentLimit); //move to constants? no
    topMotor.setControl(request);
    lc = new LaserCan(Constants.IntakeConstants.LaserCanID); 
  }


  double dist = lc.getMeasurement().distance_mm; //lasercan threshold in millimeters
  public void setState(IntakeState state) {}

  public boolean hasGamePiece() { //change to hasGamePiece
    return dist < Constants.IntakeConstants.intakeThreshhold;
  }

  public void run(double speed) {
    if(dist < Constants.IntakeConstants.intakeThreshhold) { 
      topMotor.set(speed);
    }  
  }
  
  public void outTake(double speed){
    topMotor.set(speed*-1); 
  }
  

  public void periodic(){
    topMotor.set(0.1);
  }

}