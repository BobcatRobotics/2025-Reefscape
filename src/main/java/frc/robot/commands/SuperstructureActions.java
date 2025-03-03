package frc.robot.commands;

import static edu.wpi.first.units.Units.Millimeters;

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
import frc.robot.util.IdleType;
import frc.robot.util.ScoringLevel;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class SuperstructureActions {

  /** go to the desired level's corresponding prep position, */
  public static Command prepScore(
      ScoringLevel level, BooleanSupplier flipped, Superstructure superstructure, EndEffector endEffector) {
    return superstructure.goToCoralPrepPos(level, flipped)
        .beforeStarting(
            () -> {
              superstructure.recordScoringLevel(level);
            });
  }


  /** go to the desired level's corresponding prep position, */
  public static Command score(
      Superstructure superstructure, EndEffector endEffector, IdleType endIdle) {

    // A cursed nest, six layers deep,
    // A tangled web that makes me weep.
    // Each check, another, down the chain,
    // My sanity drifts—debugging's pain.
    // Yet through this mess, it still prevails,
    // A fragile beast that somehow sails.
    // Six deep in logic, lost in despair,
    // A nested hell beyond repair.
    // If this, then that, then maybe so,
    // My patience fades with each indent’s woe.
    // Yet though it's cursed, it still must stay—
    // Refactor dreams drift far away.

    return new ConditionalCommand(
            superstructure.setState(SuperstructureState.CORAL_SCORE_L4),
            new ConditionalCommand(
                superstructure.setState(SuperstructureState.CORAL_SCORE_L3),
                new ConditionalCommand(
                    superstructure.setState(SuperstructureState.CORAL_SCORE_L2),
                    new ConditionalCommand(
                        superstructure.setState(SuperstructureState.CORAL_SCORE_L1),
                        new ConditionalCommand(
                          superstructure.setState(SuperstructureState.NET_SCORE),
                          new ConditionalCommand(
                            superstructure.setState(SuperstructureState.ALGAE_GRAB_L3),
                            superstructure.setState(SuperstructureState.ALGAE_GRAB_L2),
                             superstructure::isScoringLevelAlgaeL3),
                          superstructure::isScoringLevelNet),
                        superstructure::isScoringLevelCoralL1),
                    superstructure::isScoringLevelCoralL2),
                superstructure::isScoringLevelCoralL3),
            superstructure::isScoringLevelCoralL4)
        .andThen(
            // if were scoring in the net, outtake until we dont have a peice
            // then go to idle
            superstructure.getScoringLevel() == ScoringLevel.NET
                ? endEffector
                    .outtakeFastCommand()
                    .until(() -> endEffector.getDistanceToPiece().in(Millimeters) > 70)
                    .andThen(superstructure.setState(endIdle.state))
                // else, outake and start going down immediately
                : endEffector
                    .coralOut(superstructure.getScoringLevel())
                    .raceWith(superstructure.setState(endIdle.state)))
        .withInterruptBehavior(
            InterruptionBehavior.kCancelSelf); // TODO should this be cancel self?
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
            });
  }

  public static Command intakeAlgaeGround(Superstructure superstructure, EndEffector endEffector){
    return superstructure.setState(SuperstructureState.INTAKE_ALGAE_GROUND)
    .alongWith(endEffector.intakeAlgaeCommand())
    .andThen(superstructure.setState(SuperstructureState.IDLE_ALGAE))
    ;
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
