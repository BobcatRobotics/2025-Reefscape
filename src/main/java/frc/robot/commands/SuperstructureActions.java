package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.Superstructure.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.util.IdleType;
import frc.robot.util.ScoringLevel;

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
            superstructure.getArmRequirement(),
            superstructure.getElevatorRequirement(),
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
      ScoringLevel level, Superstructure superstructure, EndEffector endEffector) {
    Command result;
    superstructure.setScoringLevel(level);
    // determine which state to go to based on the desired level
    switch (level) {
      case L1:
        // go to the desired state
        result = superstructure.setState(SuperstructureState.CORAL_PREP_L1);
        break;
      case L2:
        result = superstructure.setState(SuperstructureState.CORAL_PREP_L2);
        break;
      case L3:
        result = superstructure.setState(SuperstructureState.CORAL_PREP_L3);
        break;
      case L4:
        result = superstructure.setState(SuperstructureState.CORAL_PREP_L4);
        break;
      case NET:
        result = superstructure.setState(SuperstructureState.NET_PREP);
        break;
      default:
        result = superstructure.setState(SuperstructureState.CORAL_PREP_L4);
    }
    result.addRequirements(
        superstructure.getArmRequirement(), superstructure.getElevatorRequirement(), endEffector);
    return result;
  }

  /** go to the desired level's corresponding prep position, */
  public static Command score(
      Superstructure superstructure, EndEffector endEffector, IdleType endIdle) {
    Command result;

    // determine which state to go to based on the desired level
    switch (superstructure.getScoringLevel()) {
      case L1:
        // go to the desired state
        result = superstructure.setState(SuperstructureState.CORAL_SCORE_L1);
        break;
      case L2:
        result = superstructure.setState(SuperstructureState.CORAL_SCORE_L2);
        break;
      case L3:
        result = superstructure.setState(SuperstructureState.CORAL_SCORE_L3);
        break;
      case L4:
        result = superstructure.setState(SuperstructureState.CORAL_SCORE_L4);
        break;
      case NET:
        result = superstructure.setState(SuperstructureState.NET_SCORE);
        break;
      default:
        result = superstructure.setState(SuperstructureState.CORAL_PREP_L4);
    }

    result.addRequirements(
        superstructure.getArmRequirement(), superstructure.getElevatorRequirement(), endEffector);

    return result
        .andThen(
            // if were scoring in the net, outtake until we dont have a peice
            // then go to idle
            superstructure.getScoringLevel() == ScoringLevel.NET
                ? endEffector
                    .outtakeCommand()
                    .until(() -> !endEffector.hasPiece())
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

  public static Command intakeCoralGround(Superstructure superstructure, CoralIntake intake) {
    return // superstructure.setState(SuperstructureState.CORAL_HANDOFF)
    Commands.run(
            () -> {
              intake.deploy();
              // intake.runIn();
            },
            intake)
        .finallyDo(
            () -> {
              intake.retract();
              // intake.dampenCoral();
            });
  }
}
