// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.util.Enums.ScoringLevel;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  public static double CORAL_IDLE_SPEED = 4;
  public static double ALGAE_IDLE_SPEED = 400;
  public static double INTAKE_CORAL_SPEED = 500;
  public static double INTAKE_ALGAE_SPEED = 500;
  public static double OUTTAKE_SPEED = -50;
  public static double OUTTAKE_FAST_SPEED = -1000;
  public static double CORAL_SCORE_SPEED = -20;

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
    return Millimeters.of(inputs.laserCanDistanceMilimeters);
  }

  public boolean hasPiece() {
    return inputs.hasPiece;
  }

  public void setSpeed(double rpm) {
    io.setSpeed(rpm);
  }

  public void idle() {
    io.setSpeed(CORAL_IDLE_SPEED);
  }

  public Command idleCoralCommand() {
    return new RunCommand(
        () -> {
          io.setSpeed(CORAL_IDLE_SPEED);
        },
        this);
  }

  public void idleCoral() {
    io.setSpeed(CORAL_IDLE_SPEED);
  }

  public Command stop() {
    return new RunCommand(
        () -> {
          io.setSpeed(0);
        },
        this);
  }

  public Command idleAlgaeCommand() {
    return new RunCommand(
        () -> {
          io.setSpeed(INTAKE_ALGAE_SPEED);
        },
        this);
  }

  public void intakeCoral() {
    io.setSpeed(INTAKE_CORAL_SPEED);
  }

  public Command intakeCoralCommand() {
    return new RunCommand(
            () -> {
              io.setSpeed(INTAKE_CORAL_SPEED);
            },
            this)
        .until(() -> inputs.hasPiece)
        .andThen(idleCoralCommand());
  }

  public Command intakeAlgaeCommand() {
    return new RunCommand(
        () -> {
          io.setSpeed(INTAKE_ALGAE_SPEED);
        },
        this);
  }

  public void outtake() {
    io.setSpeed(OUTTAKE_SPEED);
  }

  public Command outtakeCommand() {
    return new RunCommand(
        () -> {
          io.setSpeed(-10000);
        },
        this);
  }

  public Command scoreCommand(Supplier<SuperstructureState> level) {
    return Commands.run(
        () -> {
          switch (level.get()) {
            case CORAL_SCORE_L1:
              io.setSpeed(OUTTAKE_SPEED);
              break;
            case CORAL_SCORE_L2:
              io.setSpeed(CORAL_SCORE_SPEED);
              break;
            case CORAL_SCORE_L3:
              io.setSpeed(CORAL_SCORE_SPEED);
              break;
            case CORAL_SCORE_L4:
              io.setSpeed(CORAL_SCORE_SPEED);
              break;
            case POST_CORAL_SCORE_L4:
              io.setSpeed(CORAL_SCORE_SPEED);
              break;
            case NET_PREP:
              io.setSpeed(INTAKE_ALGAE_SPEED);
              break;
            case NET_SCORE:
              io.setSpeed(OUTTAKE_SPEED);
              break;
            case ALGAE_PREP_L2:
              io.setSpeed(INTAKE_ALGAE_SPEED);
              break;
            case ALGAE_PREP_L3:
              io.setSpeed(INTAKE_ALGAE_SPEED);
              break;
            default:
              io.setSpeed(CORAL_IDLE_SPEED);
              break;
          }
        },
        this);
  }

  public Command outtakeFastCommand() {
    return new RunCommand(
        () -> {
          io.setSpeed(OUTTAKE_FAST_SPEED);
        },
        this);
  }

  public Command coralOut(Supplier<ScoringLevel> level) {
    // if were scoring in l1 we need to actually shoot out the coral
    return new RunCommand(
        () -> {
          Logger.recordOutput("scoringlevel", level.get());
          if (level.get() == ScoringLevel.CORAL_L1) {
            io.setSpeed(OUTTAKE_SPEED);
          } else if (level.get() != ScoringLevel.CORAL_L4) {
            io.setSpeed(CORAL_SCORE_SPEED);
          }
        },
        this);
  }

  public Command coralOutSlow() {
    // if were scoring in l1 we need to actually shoot out the coral

    return new RunCommand(
        () -> {
          io.setSpeed(CORAL_SCORE_SPEED);
        },
        this);
  }
}
