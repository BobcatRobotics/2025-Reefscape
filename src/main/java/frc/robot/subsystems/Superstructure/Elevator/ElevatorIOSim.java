package frc.robot.subsystems.Superstructure.Elevator;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

  private DCMotor gearBox = DCMotor.getFalcon500Foc(1);
  private CANcoder encoder;
  private CANcoderSimState encoderSim;
  private ElevatorSim sim;

  public ElevatorIOSim(int encoderID) {
    encoder = new CANcoder(encoderID);
    encoderSim = encoder.getSimState();

    sim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(gearBox, encoderID, encoderID, encoderID),
            gearBox,
            encoderID,
            encoderID,
            true,
            encoderID,
            null);
  }
}
