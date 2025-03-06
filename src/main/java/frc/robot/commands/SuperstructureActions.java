package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
            });
  }

  public static Command score(
      Superstructure superstructure,
      EndEffector endEffector,
      BooleanSupplier flipped,
      BooleanSupplier shouldUseAlgae) {

    return superstructure
        .score(flipped)
        .andThen(
            new ConditionalCommand(
                endEffector
                    .intakeAlgaeCommand()
                    .until(endEffector::hasPiece)
                    .andThen(endEffector.idleAlgaeCommand()),
                endEffector
                    .coralOut(superstructure::getScoringLevel)
                    .raceWith(superstructure.setState(IdleType.UPRIGHT.state, flipped))
                    .withInterruptBehavior(InterruptionBehavior.kCancelSelf),
                shouldUseAlgae));
  }

  public static Command stow(Superstructure superstructure) {
    return superstructure.setState(SuperstructureState.RIGHT_SIDE_UP_IDLE);
  }

  public static Command stow(Superstructure superstructure, IdleType idleType) {
    return superstructure.setState(idleType.state);
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
        .setState(SuperstructureState.UPSIDE_DOWN_IDLE)
        .alongWith(
            new RunCommand(
                () -> {
                  intake.deploy(trim.get());
                  intake.runIn();
                },
                intake))
        .until(intake::hasPiece)
        .andThen(
            new ParallelDeadlineGroup(
                new WaitCommand(1),
                new RunCommand(
                    () -> {
                      intake.retract();
                    },
                    intake)))
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
        .setState(SuperstructureState.INTAKE_ALGAE_GROUND)
        .alongWith(endEffector.intakeAlgaeCommand())
        .until(endEffector::hasPiece)
        .andThen(
            superstructure
                .setState(SuperstructureState.IDLE_ALGAE)
                .alongWith(endEffector.idleAlgaeCommand()));
  }

  public static Command outtakeCoralGround(
      Superstructure superstructure, CoralIntake intake, Supplier<Angle> trim) {
    return // superstructure.setState(SuperstructureState.CORAL_HANDOFF)
    Commands.run(
            () -> {
              intake.deploy(trim.get());
              intake.setSpeed(-0.4);
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
        .setState(SuperstructureState.CORAL_HANDOFF)
        .alongWith(endEffector.intakeCoralCommand())
        .until(endEffector::hasPiece)
        .andThen(
            superstructure
                .setState(SuperstructureState.RIGHT_SIDE_UP_IDLE)
                .alongWith(endEffector.idleCoralCommand()));
  }
}
