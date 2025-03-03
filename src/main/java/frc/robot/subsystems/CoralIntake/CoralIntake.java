package frc.robot.subsystems.CoralIntake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class CoralIntake {
  private boolean deployed;

  /** Distance between minimum elevator position and top of the coral intake */
  public static final Distance CORAL_INTAKE_HEIGHT = Meters.of(0);

  public CoralIntake() {}

  public void setState(IntakeState state) {}

<<<<<<< Updated upstream
  public boolean deployed() {
    return deployed;
=======
  public void setState(IntakeState state) {
    io.setState(state);
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public void setAngle(Angle angle) {
    io.setAngle(angle);
  }

  public void deploy() {
    io.deploy();
  }

  public void retract() {
    io.retract();
  }

  public void stop() {
    io.stop();
  }

  public Command stopCommand() {
    return new InstantCommand(this::stop, this);
  }

  public void runIn() {
    io.setSpeed(0.75);
  }

  public void dampenCoral() {
    io.setSpeed(0.1);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralIntake", inputs);
    rollerDisconnectedAlert.set(!inputs.rollerMotorConnected);
    pivotDisconnectedAlert.set(!inputs.rollerMotorConnected);
>>>>>>> Stashed changes
  }
}
