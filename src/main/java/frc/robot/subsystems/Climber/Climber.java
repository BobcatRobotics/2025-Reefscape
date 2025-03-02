package frc.robot.subsystems.Climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private ClimberIO io;
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert =
      new Alert("Climber motor disconnected!", AlertType.kWarning);

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    motorDisconnectedAlert.set(!inputs.connected);
  }

  public void setDutyCycle(double output) {
    io.setDutyCycle(output);
  }

  public void setPosition(Rotation2d pos) {
    io.setPosition(pos);
  }
}
