package frc.robot.commands;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.Superstructure.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.util.Enums.IdleType;
import frc.robot.util.Enums.ScoringLevel;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class SuperstructureActions {

  /** go to the desired level's corresponding prep position, */
  public static Command prepScore(
      ScoringLevel level,
      BooleanSupplier flipped,
      Superstructure superstructure,
      EndEffector endEffector) {
    return superstructure
        .goToPrepPos(level, flipped)
        .beforeStarting(
            () -> {
              superstructure.recordScoringLevel(level);
              superstructure.setLastPrepPosition(level.prepState);
            });
  }

  public static Command retractFromPlace(
      Superstructure superstructure,
      EndEffector endEffector,
      BooleanSupplier shouldUseAlgae,
      BooleanSupplier flipped) {
    return new ConditionalCommand(
        endEffector.outtakeFastCommand().until(() -> !endEffector.hasPiece()),
        endEffector
            .coralOut(superstructure::getScoringLevel)
            .raceWith(superstructure.setState(IdleType.UPRIGHT.state, flipped)),
        shouldUseAlgae);
  }

  public static Command stow(Superstructure superstructure) {
    return superstructure.setState(SuperstructureState.RIGHT_SIDE_UP_IDLE, () -> false);
  }

  public static Command stow(Superstructure superstructure, IdleType idleType) {
    return superstructure.setState(idleType.state, () -> false);
  }

  /**
   * Picks up a peice that is indexed in the carwash
   *
   * <p>This command assumes that the peice is already there, it will NOT check to see if a peice is
   * indexed
   *
   * <p>After it picks up the peice, the arm will flip around to the coral idle spot
   */
  public static Command intakeCoralGround(
      Superstructure superstructure, CoralIntake intake, Supplier<Angle> trim) {
    return superstructure
        .setState(SuperstructureState.HANDOFF_PREP, () -> false)
        .alongWith(
            new RunCommand(
                () -> {
                  intake.deploy(trim.get());
                  intake.runIn();
                },
                intake))
        .finallyDo(
            () -> {
              intake.retract();
              intake.stop();
            })
        .handleInterrupt(
            () -> {
              intake.dampenCoral();
              intake.retract();
            });
  }

  public static Command intakeAlgaeGround(Superstructure superstructure, EndEffector endEffector) {
    return superstructure
        .setState(SuperstructureState.INTAKE_ALGAE_GROUND, () -> false)
        .alongWith(endEffector.intakeAlgaeCommand())
        .until(endEffector::hasPiece)
        .andThen(endEffector.idleAlgaeCommand());
  }

  public static Command outtakeCoralGround(
      Superstructure superstructure, CoralIntake intake, Supplier<Angle> trim) {
    return // superstructure.setState(SuperstructureState.CORAL_HANDOFF)
    Commands.run(
            () -> {
              intake.deploy(trim.get());
              intake.setSpeed(Amps.of(-30));
            },
            intake)
        .finallyDo(
            () -> {
              intake.retract();
              intake.stop();
            });
  }

  public static Command handoff(Superstructure superstructure, EndEffector endEffector) {
    return superstructure
        .setState(SuperstructureState.CORAL_HANDOFF, () -> false)
        .alongWith(endEffector.intakeCoralCommand())
        .until(endEffector::hasPiece)
        .andThen(
            superstructure
                .setState(SuperstructureState.RIGHT_SIDE_UP_IDLE, () -> false)
                .alongWith(endEffector.idleCoralCommand()));
  }

  public static Command handoffThenPrepL4(Superstructure superstructure, EndEffector endEffector) {
    return superstructure
        .setState(SuperstructureState.CORAL_HANDOFF, () -> false)
        .alongWith(endEffector.intakeCoralCommand())
        .until(endEffector::hasPiece)
        .withTimeout(RobotBase.isSimulation() ? 0.25 : 0.75)
        .andThen(
            superstructure
                .setState(SuperstructureState.CORAL_PREP_L4, () -> false)
                .alongWith(endEffector.idleCoralCommand()));
  }

  public static Command handoffThenPrepL4Auto(
      Superstructure superstructure, EndEffector endEffector) {
    return superstructure
        .setState(SuperstructureState.CORAL_HANDOFF, () -> false)
        .alongWith(endEffector.intakeCoralCommand())
        .until(endEffector::hasPiece)
        .withTimeout(RobotBase.isSimulation() ? 0.25 : 0.75)
        .andThen(
            new ConditionalCommand(
                superstructure
                    .setState(SuperstructureState.CORAL_PREP_L4, () -> false)
                    .alongWith(endEffector.idleCoralCommand()),
                superstructure
                    .setState(SuperstructureState.UPSIDE_DOWN_IDLE, () -> false)
                    .alongWith(endEffector.idleCoralCommand()),
                () -> endEffector.hasPiece()));
  }

  public static Command handoffNoIdle(Superstructure superstructure, EndEffector endEffector) {
    return superstructure
        .setState(SuperstructureState.CORAL_HANDOFF, () -> false)
        .alongWith(endEffector.intakeCoralCommand())
        .until(endEffector::hasPiece)
        .andThen(
            superstructure
                .setState(SuperstructureState.RIGHT_SIDE_UP_IDLE, () -> false)
                .raceWith(endEffector.idleCoralCommand()));
  }
}
