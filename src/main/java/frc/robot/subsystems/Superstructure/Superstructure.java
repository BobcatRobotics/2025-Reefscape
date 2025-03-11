package frc.robot.subsystems.Superstructure;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure.Arm.Arm;
import frc.robot.subsystems.Superstructure.Elevator.Elevator;
import frc.robot.util.Enums.ScoringLevel;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public static final Distance ELEVATOR_TOLERANCE = Inches.of(0.5);
  public static final Rotation2d ARM_TOLERANCE = Rotation2d.fromDegrees(1.5);
  private final StateGraph graph = StateGraph.getInstance();

  private SuperstructureState currentState = SuperstructureState.UNKNOWN;
  private Arm arm;
  private Elevator elevator;
  private ScoringLevel scoringLevel = ScoringLevel.CORAL_L1;
  private SuperstructureState lastPrepPosition = SuperstructureState.RIGHT_SIDE_UP_IDLE;

  private final RobotVisualizer visualizer = RobotVisualizer.getInstance();

  private boolean isScoring = false;

  /** A class representing the Arm-Elevator superstructure */
  public Superstructure(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;
  }

  /**
   * @return the position of the elevator, expressed as a percentage of its max travel, [0,1]
   */
  public double getElevatorPercentage() {
    return elevator.positionPercent();
  }

  public void recordScoringLevel(ScoringLevel level) {
    Logger.recordOutput("Superstructure/DesiredScoringLevel", level);
    scoringLevel = level;
  }

  public void setLastPrepPosition(SuperstructureState state) {
    lastPrepPosition = state;
  }

  public SuperstructureState getLastPrepPosition() {
    return lastPrepPosition;
  }

  /** only for use in commands */
  public ScoringLevel getScoringLevel() {
    Logger.recordOutput("Superstructure/gotDesiredScoringLevel", scoringLevel);

    return scoringLevel;
  }

  public boolean isAlgaeScoringLevel() {
    return getScoringLevel() == ScoringLevel.ALGAE_L2 || getScoringLevel() == ScoringLevel.ALGAE_L3;
  }

  public boolean isScoringLevelCoralL1() {
    return getScoringLevel() == ScoringLevel.CORAL_L1;
  }

  public boolean isScoringLevelCoralL2() {
    return getScoringLevel() == ScoringLevel.CORAL_L2;
  }

  public boolean isScoringLevelCoralL3() {
    return getScoringLevel() == ScoringLevel.CORAL_L3;
  }

  public boolean isScoringLevelCoralL4() {
    return getScoringLevel() == ScoringLevel.CORAL_L4;
  }

  public boolean isScoringLevelNet() {
    return getScoringLevel() == ScoringLevel.NET;
  }

  public boolean isScoringLevelAlgaeL2() {
    return getScoringLevel() == ScoringLevel.ALGAE_L2;
  }

  public boolean isScoringLevelAlgaeL3() {
    return getScoringLevel() == ScoringLevel.ALGAE_L2;
  }

  public SuperstructureState getState() {
    return currentState;
  }

  public Command setState(SuperstructureState goal, BooleanSupplier hasPeice) {

    return Commands.run(
            () -> {
              visualizer.setDesiredSuperstructureState(goal);
              Logger.recordOutput("Superstructure/CurrentState", currentState);
              Logger.recordOutput("Superstructure/DesiredState", goal);
              SuperstructureState nextState = nextStateInPath(currentState, goal);
              Logger.recordOutput("Superstructure/NextState", nextState);

              if (arm.getDesiredState() != goal.armState) {
                arm.setState(nextState.armState, false, hasPeice.getAsBoolean());
              }
              if (elevator.getDesiredState() != goal.elevatorState) {

                elevator.setState(nextState.elevatorState);
              }

              if (superstructureInTolerance(nextState)) {
                currentState = nextState;
              }
            },
            arm,
            elevator)
        .until(() -> getState() == goal)
        .finallyDo(
            () -> {
              Logger.recordOutput("Superstructure/CurrentState", currentState);
            });
  }

  public Command setState(Supplier<SuperstructureState> goal, BooleanSupplier hasPeice) {

    return Commands.run(
            () -> {
              visualizer.setDesiredSuperstructureState(goal.get());
              Logger.recordOutput("Superstructure/CurrentState", currentState);
              Logger.recordOutput("Superstructure/DesiredState", goal.get());
              SuperstructureState nextState = nextStateInPath(currentState, goal.get());
              Logger.recordOutput("Superstructure/NextState", nextState);

              if (arm.getDesiredState() != goal.get().armState) {
                arm.setState(nextState.armState, false, hasPeice.getAsBoolean());
              }
              if (elevator.getDesiredState() != goal.get().elevatorState) {

                elevator.setState(nextState.elevatorState);
              }

              if (superstructureInTolerance(nextState)) {
                currentState = nextState;
              }
            },
            arm,
            elevator)
        .until(() -> getState() == goal.get())
        .finallyDo(
            () -> {
              Logger.recordOutput("Superstructure/CurrentState", currentState);
            });
  }

  public Command setState(
      SuperstructureState goal, BooleanSupplier armFlipped, BooleanSupplier hasPeice) {

    return Commands.run(
            () -> {
              Logger.recordOutput("Superstructure/IsFlippingArm", armFlipped);
              visualizer.setDesiredSuperstructureState(goal);
              Logger.recordOutput("Superstructure/CurrentState", currentState);
              Logger.recordOutput("Superstructure/DesiredState", goal);
              SuperstructureState nextState = nextStateInPath(currentState, goal);
              Logger.recordOutput("Superstructure/NextState", nextState);

              if ((arm.getDesiredState() != goal.armState
                  || (arm.isFlipped() != armFlipped.getAsBoolean()))) {
                arm.setState(
                    nextState.armState, armFlipped.getAsBoolean(), hasPeice.getAsBoolean());
              }
              if (elevator.getDesiredState() != goal.elevatorState) {

                elevator.setState(nextState.elevatorState);
              }

              if (superstructureInTolerance(nextState)) {
                currentState = nextState;
              }
            },
            arm,
            elevator)
        .until(() -> getState() == goal)
        .finallyDo(
            () -> {
              Logger.recordOutput("Superstructure/CurrentState", currentState);
            });
  }

  private SuperstructureState getDesiredScoringState() {
    SuperstructureState goal;

    switch (getScoringLevel()) {
      case CORAL_L1:
        goal = SuperstructureState.CORAL_SCORE_L1;
        break;
      case CORAL_L2:
        goal = SuperstructureState.CORAL_SCORE_L2;
        break;
      case CORAL_L3:
        goal = SuperstructureState.CORAL_SCORE_L3;
        break;
      case CORAL_L4:
        goal = SuperstructureState.POST_CORAL_SCORE_L4;
        break;
      case ALGAE_L2:
        goal = SuperstructureState.ALGAE_PREP_L2;
        break;
      case ALGAE_L3:
        goal = SuperstructureState.ALGAE_PREP_L3;
        break;
      case NET:
        goal = SuperstructureState.NET_SCORE;
      default:
        goal = SuperstructureState.NET_SCORE;
    }

    Logger.recordOutput("wtf", goal);

    return goal;
  }

  private SuperstructureState getDesiredScoringState(ScoringLevel level) {
    SuperstructureState goal;

    switch (level) {
      case CORAL_L1:
        goal = SuperstructureState.CORAL_SCORE_L1;
        break;
      case CORAL_L2:
        goal = SuperstructureState.CORAL_SCORE_L2;
        break;
      case CORAL_L3:
        goal = SuperstructureState.CORAL_SCORE_L3;
        break;
      case CORAL_L4:
        goal = SuperstructureState.CORAL_SCORE_L4;
        break;
      case ALGAE_L2:
        goal = SuperstructureState.ALGAE_PREP_L2;
        break;
      case ALGAE_L3:
        goal = SuperstructureState.ALGAE_PREP_L3;
        break;
      case NET:
        goal = SuperstructureState.NET_SCORE;
      default:
        goal = SuperstructureState.CORAL_SCORE_L4;
    }
    return goal;
  }

  public Command score(
      BooleanSupplier shouldFlipArm, ScoringLevel level, BooleanSupplier hasPeice) {

    return Commands.run(
            () -> {
              SuperstructureState goal = getDesiredScoringState(level);
              boolean armFlipped = shouldFlipArm.getAsBoolean();

              Logger.recordOutput("Superstructure/IsFlippingArm", armFlipped);
              visualizer.setDesiredSuperstructureState(goal);
              Logger.recordOutput("Superstructure/CurrentState", currentState);
              Logger.recordOutput("Superstructure/DesiredState", goal);
              SuperstructureState nextState = nextStateInPath(currentState, goal);
              Logger.recordOutput("Superstructure/NextState", nextState);

              if (arm.getDesiredState() != goal.armState || (arm.isFlipped() != armFlipped)) {
                arm.setState(nextState.armState, armFlipped, hasPeice.getAsBoolean());
              }
              if (elevator.getDesiredState() != goal.elevatorState) {
                elevator.setState(nextState.elevatorState);
              }

              if (superstructureInTolerance(nextState)) {
                currentState = nextState;
              }
            },
            arm,
            elevator)
        .until(() -> getState() == getDesiredScoringState(level))
        .finallyDo(
            () -> {
              Logger.recordOutput("Superstructure/CurrentState", currentState);
            });
  }

  public Command score(BooleanSupplier shouldFlipArm, BooleanSupplier hasPeice) {
    return Commands.run(
            () -> {
              isScoring = true;
              SuperstructureState goal = getDesiredScoringState();
              boolean armFlipped = shouldFlipArm.getAsBoolean();
              Logger.recordOutput("desiredscoringstate", getDesiredScoringState());

              Logger.recordOutput("Superstructure/IsFlippingArm", armFlipped);
              visualizer.setDesiredSuperstructureState(goal);
              Logger.recordOutput("Superstructure/CurrentState", currentState);
              Logger.recordOutput("Superstructure/DesiredState", goal);
              SuperstructureState nextState = nextStateInPath(currentState, goal);
              Logger.recordOutput("Superstructure/NextState", nextState);

              if (arm.getDesiredState() != goal.armState || (arm.isFlipped() != armFlipped)) {
                arm.setState(nextState.armState, armFlipped, hasPeice.getAsBoolean());
              }
              if (elevator.getDesiredState() != goal.elevatorState) {
                elevator.setState(nextState.elevatorState);
              }

              if (superstructureInTolerance(nextState)) {
                currentState = nextState;
              }
            },
            arm,
            elevator)
        .until(() -> getState() == getDesiredScoringState())
        .finallyDo(
            () -> {
              Logger.recordOutput("Superstructure/CurrentState", currentState);
            });
  }

  public Command gotToLastPrepPosition(BooleanSupplier hasPiece) {
    return setState(this::getLastPrepPosition, hasPiece)
        .alongWith(new InstantCommand(() -> isScoring = false));
  }

  public boolean isScoring() {
    return isScoring;
  }

  public Command scoreL4Auto(BooleanSupplier shouldFlipArm, BooleanSupplier hasPeice) {

    return Commands.run(
            () -> {
              SuperstructureState goal = SuperstructureState.AUTO_CORAL_SCORE_L4;
              boolean armFlipped = shouldFlipArm.getAsBoolean();

              Logger.recordOutput("Superstructure/IsFlippingArm", armFlipped);
              visualizer.setDesiredSuperstructureState(goal);
              Logger.recordOutput("Superstructure/CurrentState", currentState);
              Logger.recordOutput("Superstructure/DesiredState", goal);
              SuperstructureState nextState = nextStateInPath(currentState, goal);
              Logger.recordOutput("Superstructure/NextState", nextState);

              if (arm.getDesiredState() != goal.armState || (arm.isFlipped() != armFlipped)) {
                arm.setState(nextState.armState, armFlipped, hasPeice.getAsBoolean());
              }
              if (elevator.getDesiredState() != goal.elevatorState) {
                elevator.setState(nextState.elevatorState);
              }

              if (superstructureInTolerance(nextState)) {
                currentState = nextState;
              }
            },
            arm,
            elevator)
        .until(() -> getState() == SuperstructureState.AUTO_CORAL_SCORE_L4)
        .finallyDo(
            () -> {
              Logger.recordOutput("Superstructure/CurrentState", currentState);
            });
  }

  /**
   * Performs a breadth-first search of all possible states to find the shortest path from the start
   * state to the goal state.
   *
   * @param start Current state
   * @param goal desired goal state
   * @return A list of states representing the shortest path from start to goal, {@code null} if no
   *     path exists.
   */
  public List<SuperstructureState> findShortestPath(
      SuperstructureState start, SuperstructureState goal) {

    // Queue to manage the paths to be explored. Each element in the queue is a list
    // of states
    // representing a path.
    Queue<List<SuperstructureState>> queue = new LinkedList<>();

    // Set to keep track of visited states to avoid revisiting them and getting
    // stuck in loops.
    Set<SuperstructureState> visited = new HashSet<>();

    // Start BFS with a path containing only the start node.
    // The initial path is a singleton list containing just the start state.
    queue.add(Collections.singletonList(start));

    // Continue exploring until there are no more paths to explore in the queue.
    while (!queue.isEmpty()) {
      // Retrieve and remove the first path from the queue.
      List<SuperstructureState> path = queue.poll();

      // Get the last state in the current path. This is the state we will explore
      // next.
      SuperstructureState lastState = path.get(path.size() - 1);

      // Check if the last state in the current path is the goal state.
      if (lastState.equals(goal)) {
        StringBuilder pathString = new StringBuilder("Transition: ");
        for (SuperstructureState state : path) {
          pathString.append(state.name()).append(", ");
        }
        Logger.recordOutput("Superstructure/StatePath", pathString.toString());
        // If it is, return the current path as it is the shortest path found.
        return path;
      }

      // If the last state has not been visited yet, process it.
      if (!visited.contains(lastState)) {
        // Mark the last state as visited to avoid processing it again in the future.
        visited.add(lastState);

        // Explore all neighboring states of the last state.
        for (SuperstructureState neighbor : graph.getNeighbors(lastState)) {
          // Create a new path by copying the current path and adding the neighbor state
          // to it.
          List<SuperstructureState> newPath = new ArrayList<>(path);
          newPath.add(neighbor);

          // Add the new path to the queue for further exploration.
          queue.add(newPath);
        }
      }
    }

    // If the queue is exhausted and no path to the goal state is found, return
    // null.
    return null;
  }

  public SuperstructureState nextStateInPath(
      SuperstructureState current, SuperstructureState goal) {
    List<SuperstructureState> path = findShortestPath(current, goal);
    // If there's no path or we're already at the goal, return current.
    if (path == null || path.size() < 2) {
      return current;
    }
    // Otherwise, the next state along the optimal path is at index 1.
    return path.get(1);
  } // /**
  // * @return {@code False} if the elevator is high enough where the arm can't
  // collide with the
  // * intake
  // */
  // public static boolean checkForArmCollision(ArmZone zone, ElevatorState
  // elevatorState) {
  // if (isInIntakeZone(zone)) {
  // return elevatorState.pos.getRotations() <
  // Elevator.MIN_HEIGHT_INTAKE_AVOIDANCE.getRotations();
  // } else if (zone == ArmZone.BOTTOM_ZONE) {
  // return elevatorState.pos.getRotations() <
  // Elevator.MIN_HEIGHT_BOTTOM_AVOIDANCE.getRotations();
  // } else {
  // return false;
  // }
  // }

  // public static boolean checkForArmCollision(ArmState armState, ElevatorState
  // elevatorState) {
  // return checkForArmCollision(armState.zone, elevatorState);
  // }

  /**
   * @return {@code true} if the arm and elevator are within the tolerances for their current states
   */
  public boolean superstructureInTolerance() {
    return elevator.inTolerance() && arm.inTolerance();
  }

  // /**
  // * @param zone
  // * @return {@code true} if the given armzone is the coral or algae zone
  // */
  // public static boolean isInIntakeZone(ArmZone zone) {
  // return zone == ArmZone.CORAL_INTAKE || zone == ArmZone.ALGAE_INTAKE;
  // }

  // /** see Assets\Docs\TopUpperLimit.png */
  // public static ArmZone getArmZone(Rotation2d position) {
  // double deg = position.getDegrees();
  // if (deg >= Arm.TOP_LOWER_LIMIT.getDegrees() && deg <=
  // Arm.TOP_UPPER_LIMIT.getDegrees()) {
  // return ArmZone.TOP_ZONE;
  // } else if (deg > Arm.TOP_UPPER_LIMIT.getDegrees()
  // && deg < Arm.BOTTOM_LOWER_LIMIT.getDegrees()) {
  // return ArmZone.CORAL_INTAKE;
  // } else if (deg > Arm.BOTTOM_LOWER_LIMIT.getDegrees()
  // && deg < Arm.BOTTOM_UPPER_LIMIT.getDegrees()) {
  // return ArmZone.BOTTOM_ZONE;
  // } else {
  // return ArmZone.ALGAE_INTAKE;
  // }
  // }

  @AutoLogOutput(key = "Superstructure/inTolerance")
  public boolean superstructureInTolerance(SuperstructureState goal) {
    return arm.inTolerance(goal) && elevator.inTolerance(goal);
  }

  public Command goToPrepPos(ScoringLevel level, BooleanSupplier flipped) {
    switch (level) {
      case CORAL_L1:
        return setState(SuperstructureState.CORAL_PREP_L1, flipped);
      case CORAL_L2:
        return setState(SuperstructureState.CORAL_PREP_L2, flipped);
      case CORAL_L3:
        return setState(SuperstructureState.CORAL_PREP_L3, flipped);
      case CORAL_L4:
        return setState(SuperstructureState.CORAL_PREP_L4, flipped);
      case NET:
        return setState(SuperstructureState.NET_PREP, flipped);
      case ALGAE_L2:
        return setState(SuperstructureState.ALGAE_PREP_L2, flipped);
      case ALGAE_L3:
        return setState(SuperstructureState.ALGAE_PREP_L3, flipped);
      default:
        return setState(SuperstructureState.CORAL_PREP_L1, flipped);
    }
  }

  public static boolean isInPrepState(SuperstructureState desiredState) {
    switch (desiredState) {
      case CORAL_PREP_L1:
        return true;
      case CORAL_PREP_L2:
        return true;
      case CORAL_PREP_L3:
        return true;
      case CORAL_PREP_L4:
        return true;
      default:
        return false;
    }
  }

  public boolean isInPrepState() {
    return isInPrepState(getDesiredScoringState()) || isInPrepState(getState());
  }
}
