package frc.robot.commands;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.Superstructure.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.util.IdleType;
import frc.robot.util.ScoringLevel;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SuperstructureActions {
  /**
   * Picks up a peice that is indexed in the carwash
   *
   * <p>This command assumes that the peice is already there, it will NOT check to see if a peice is
   * indexed
   *
   * <p>After it picks up the peice, the arm will flip around to the coral idle spot
   *
   * <p>END STATE: RIGHT_SIDE_UP_IDLE
   */
  public static Command intakeFromGround(Superstructure superstructure, EndEffector endEffector) {
    return Commands.run(
            // go to ground intake prep state if were not there already
            () -> {
              superstructure.setState(SuperstructureState.UPSIDE_DOWN_IDLE);
            },
            endEffector)
        .andThen(
            // afterwards, start running the intake
            endEffector
                .intakeCommand()
                // and go to the handoff position
                .alongWith(
                    superstructure
                        .setState(SuperstructureState.CORAL_HANDOFF)
                        // untill the end effector has the peice or 5 seconds pass
                        .until(() -> endEffector.hasPiece())
                        .withTimeout(5)))
        // once we have a peice, go to the idle with coral position
        .andThen(superstructure.setState(SuperstructureState.RIGHT_SIDE_UP_IDLE));
  }

  /** go to the desired level's corresponding prep position, */
  public static Command prepScore(
      ScoringLevel level, boolean flipped, Superstructure superstructure, EndEffector endEffector) {
    return superstructure
        .setState(getCoralScoringPos(level, flipped))
        .beforeStarting(
            () -> {
              superstructure.recordScoringLevel(level);
            });
  }

  public static SuperstructureState getCoralScoringPos(ScoringLevel level, boolean flipped) {
    // determine which state to go to based on the desired level
    switch (level) {
      case L1:
        return flipped
            ? SuperstructureState.FLIPPED_CORAL_PREP_L1
            : SuperstructureState.CORAL_PREP_L1;
      case L2:
        return flipped
            ? SuperstructureState.FLIPPED_CORAL_PREP_L2
            : SuperstructureState.CORAL_PREP_L2;
      case L3:
        return flipped
            ? SuperstructureState.FLIPPED_CORAL_PREP_L3
            : SuperstructureState.CORAL_PREP_L3;
      case L4:
        return flipped
            ? SuperstructureState.FLIPPED_CORAL_PREP_L4
            : SuperstructureState.CORAL_PREP_L4;
      case NET:
        return flipped
            ? SuperstructureState.FLIPPED_NET_PREP
            : SuperstructureState.FLIPPED_NET_PREP;
      default:
        return flipped
            ? SuperstructureState.FLIPPED_CORAL_PREP_L4
            : SuperstructureState.CORAL_PREP_L4;
    }
  }

  /** go to the desired level's corresponding prep position, */
  public static Command score(
      Superstructure superstructure, EndEffector endEffector, IdleType endIdle) {

    // A cursed nest, four layers deep,
    // A tangled web that makes me weep.
    // Each check, another, down the chain,
    // My sanity drifts—debugging's pain.
    // Yet through this mess, it still prevails,
    // A fragile beast that somehow sails.
    // Four deep in logic, lost in despair,
    // A nested hell beyond repair.
    // If this, then that, then maybe so,
    // My patience fades with each indent’s woe.
    // Yet though it's cursed, it still must stay—
    // Refactor dreams drift far away.

    return new ConditionalCommand(
            superstructure
                .setState(SuperstructureState.CORAL_SCORE_L4)
                .alongWith(
                    new InstantCommand(
                        () -> {
                          Logger.recordOutput("hmm", Timer.getFPGATimestamp());
                        })),
            new ConditionalCommand(
                superstructure.setState(SuperstructureState.CORAL_SCORE_L3),
                new ConditionalCommand(
                    superstructure.setState(SuperstructureState.CORAL_SCORE_L2),
                    new ConditionalCommand(
                        superstructure.setState(SuperstructureState.CORAL_SCORE_L1),
                        superstructure.setState(SuperstructureState.NET_SCORE),
                        superstructure::isScoringLevelL1),
                    superstructure::isScoringLevelL2),
                superstructure::isScoringLevelL3),
            superstructure::isScoringLevelL4)
        .andThen(
            // if were scoring in the net, outtake until we dont have a peice
            // then go to idle
            superstructure.getScoringLevel() == ScoringLevel.NET
                ? endEffector
                    .outtakeCommand()
                    .until(() -> endEffector.getDistanceToPiece().in(Millimeters) > 70)
                    .andThen(superstructure.setState(endIdle.state))
                // else, outake and start going down immediately
                : endEffector.outtakeCommand().raceWith(superstructure.setState(endIdle.state)))
        .withInterruptBehavior(
            InterruptionBehavior.kCancelSelf); // TODO should this be cancel self?
  }

  public static Command stow(Superstructure superstructure) {
    return superstructure.setState(SuperstructureState.RIGHT_SIDE_UP_IDLE);
  }

  public static Command stow(Superstructure superstructure, IdleType idleType) {
    return superstructure.setState(idleType.state);
  }

  public static Command intakeCoralGround(
      Superstructure superstructure, CoralIntake intake, Supplier<Angle> trim) {
    return
    // .setState(SuperstructureState.UPSIDE_DOWN_IDLE)
    new RunCommand(
            () -> {
              intake.deploy(trim.get());
              intake.runIn();
            },
            intake)
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
        .alongWith(endEffector.intakeCommand())
        .until(endEffector::hasPiece);
    // .andThen(superstructure.setState(SuperstructureState.UPSIDE_DOWN_IDLE));
  }
}
