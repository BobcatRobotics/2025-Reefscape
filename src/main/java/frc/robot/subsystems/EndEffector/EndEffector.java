// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  public static double IDLE_SPEED = 60;
  public static double INTAKE_SPEED = 1000;
  public static double OUTTAKE_SPEED = -300;

  private EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
  private EndEffectorIO io;

  private final Alert motorDisconnectedAlert =
      new Alert("End Effector motor disconnected!", AlertType.kWarning);

  /** Creates a new EndEffector. */
  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);
    motorDisconnectedAlert.set(!inputs.motorConnected);
  }

  public Distance getDistanceToPiece() {
    return Meters.of(inputs.laserCanDistanceMilimeters);
  }

  public boolean hasPiece() {
    return inputs.hasPiece;
  }

  public void setSpeed(double rpm) {
    io.setSpeed(rpm);
  }

  public void idle() {
    io.setSpeed(IDLE_SPEED);
  }

  public Command idleCommand() {
    return new RunCommand(
        () -> {
          io.setSpeed(IDLE_SPEED);
        },
        this);
  }

  public void intake() {
    io.setSpeed(INTAKE_SPEED);
  }

  public Command intakeCommand() {
    return new RunCommand(
            () -> {
              io.setSpeed(INTAKE_SPEED);
            },
            this)
        .until(() -> inputs.hasPiece);
  }

  public void outtake() {
    io.setSpeed(OUTTAKE_SPEED);
  }

  public Command outtakeCommand() {
    return new RunCommand(
        () -> {
          io.setSpeed(OUTTAKE_SPEED);
        },
        this);
  }
}
