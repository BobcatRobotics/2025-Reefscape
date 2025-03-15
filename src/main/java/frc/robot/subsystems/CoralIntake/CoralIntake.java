package frc.robot.subsystems.CoralIntake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.RobotVisualizer;

import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
  public static double GEAR_RATIO = 33.98 / 1; // TODO find this!

  /** the mazimum travel distance of the elevator */
  public static final Rotation2d RANGE_OF_MOTION = Rotation2d.fromDegrees(125);
  public static final double DEGREES_PER_ROTATION = RANGE_OF_MOTION.getDegrees() / CoralIntakeIOTalonFX.MAX_ROTATIONS; //TODO verify
  private final Alert rollerDisconnectedAlert = new Alert("Intake roller motor disconnected!", AlertType.kWarning);
  private final Alert pivotDisconnectedAlert = new Alert("Intake pivot motor disconnected!", AlertType.kWarning);

  private CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();
  private CoralIntakeIO io;
  private RobotVisualizer visualizer = RobotVisualizer.getInstance();

  public CoralIntake(CoralIntakeIO io) {
    this.io = io;
  }

  public void setState(IntakeState state) {
    io.setState(state);
  }

  public void setSpeed(Current speed) {
    io.setSpeed(speed);
  }

  public void setAngle(Angle angle) {
    io.setAngle(angle);
  }

  public void deploy(Angle trim) {
    io.deploy(trim);
  }

  public void deploy() {
    io.deploy(Rotations.of(0));
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
    io.setSpeed(Amps.of(40));
  }

  public void dampenCoral() {
    io.setSpeed(Amps.of(12));
  }

  public boolean hasPiece() {
    return inputs.hasPiece;
  }

  public void zeroPosition() {
    io.zeroPosition();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralIntake", inputs);
    rollerDisconnectedAlert.set(!inputs.rollerMotorConnected);
    pivotDisconnectedAlert.set(!inputs.rollerMotorConnected);
    visualizer.setIntakeRotation(
      Rotation2d.fromDegrees(inputs.positionRotations *DEGREES_PER_ROTATION)
    );

  }
}
