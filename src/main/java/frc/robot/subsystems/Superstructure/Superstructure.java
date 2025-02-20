package frc.robot.subsystems.Superstructure;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Superstructure.Arm.Arm;
import frc.robot.subsystems.Superstructure.Elevator.Elevator;
import frc.robot.util.ScoringLevel;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Set;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public static final Distance ELEVATOR_TOLERANCE = Inches.of(0.5);
  public static final Rotation2d ARM_TOLERANCE = Rotation2d.fromDegrees(1.5);
  private final StateGraph graph = StateGraph.getInstance();


  private SuperstructureState currentState = SuperstructureState.UNKNOWN;
  private Arm arm;
  private Elevator elevator;
  private ScoringLevel scoringLevel = ScoringLevel.L1;
  

  private SuperstructureState desiredState = SuperstructureState.UNKNOWN;
  private SuperstructureState desiredTransitionState = SuperstructureState.UNKNOWN;

  /** A class representing the Arm-Elevator superstructure */
  public Superstructure(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;
  }

  public void setScoringLevel(ScoringLevel level){
    Logger.recordOutput("Auto/DesiredScoringLevel", level);
    scoringLevel = level;
  }
  /**
   * only for use in commands
   */
  public ScoringLevel getScoringLevel(){
    return scoringLevel;
  }

  /**
   * ONLY USE FOR DECLARING COMMAND REQUIREMENTS!!! 
   */
  public Arm getArmRequirement(){
    return arm;
  }
  /**
   * ONLY USE FOR DECLARING COMMAND REQUIREMENTS!!! 
   */
  public Elevator getElevatorRequirement(){
    return elevator;
  }

  public SuperstructureState getState() {
    return currentState;
  }

  public SequentialCommandGroup setState(SuperstructureState state) {
    List<SuperstructureState> states = findShortestPath(getState(), state);
    if (states == null) {
      return new SequentialCommandGroup();
    }
    SequentialCommandGroup result = new SequentialCommandGroup();

    for (SuperstructureState desiredState : states) {
      if (desiredState != currentState) {
        result.addCommands(setSingleState(desiredState));
      }
    }
    return result;
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

    // Queue to manage the paths to be explored. Each element in the queue is a list of states
    // representing a path.
    Queue<List<SuperstructureState>> queue = new LinkedList<>();

    // Set to keep track of visited states to avoid revisiting them and getting stuck in loops.
    Set<SuperstructureState> visited = new HashSet<>();

    // Start BFS with a path containing only the start node.
    // The initial path is a singleton list containing just the start state.
    queue.add(Collections.singletonList(start));

    // Continue exploring until there are no more paths to explore in the queue.
    while (!queue.isEmpty()) {
      // Retrieve and remove the first path from the queue.
      List<SuperstructureState> path = queue.poll();

      // Get the last state in the current path. This is the state we will explore next.
      SuperstructureState lastState = path.get(path.size() - 1);

      // Check if the last state in the current path is the goal state.
      if (lastState.equals(goal)) {
        // If it is, return the current path as it is the shortest path found.
        return path;
      }

      // If the last state has not been visited yet, process it.
      if (!visited.contains(lastState)) {
        // Mark the last state as visited to avoid processing it again in the future.
        visited.add(lastState);

        // Explore all neighboring states of the last state.
        for (SuperstructureState neighbor : graph.getNeighbors(lastState)) {
          // Create a new path by copying the current path and adding the neighbor state to it.
          List<SuperstructureState> newPath = new ArrayList<>(path);
          newPath.add(neighbor);

          // Add the new path to the queue for further exploration.
          queue.add(newPath);
        }
      }
    }

    // If the queue is exhausted and no path to the goal state is found, return null.
    return null;
  }

  // /**
  //  * @return {@code False} if the elevator is high enough where the arm can't collide with the
  //  *     intake
  //  */
  // public static boolean checkForArmCollision(ArmZone zone, ElevatorState elevatorState) {
  //   if (isInIntakeZone(zone)) {
  //     return elevatorState.pos.getRotations() <
  // Elevator.MIN_HEIGHT_INTAKE_AVOIDANCE.getRotations();
  //   } else if (zone == ArmZone.BOTTOM_ZONE) {
  //     return elevatorState.pos.getRotations() <
  // Elevator.MIN_HEIGHT_BOTTOM_AVOIDANCE.getRotations();
  //   } else {
  //     return false;
  //   }
  // }

  // public static boolean checkForArmCollision(ArmState armState, ElevatorState elevatorState) {
  //   return checkForArmCollision(armState.zone, elevatorState);
  // }

  /**
   * @return {@code true} if the arm and elevator are within the tolerances for their current states
   */
  public boolean superstructureInTolerance() {
    return elevator.inTolerance() && arm.inTolerance();
  }

  // /**
  //  * @param zone
  //  * @return {@code true} if the given armzone is the coral or algae zone
  //  */
  // public static boolean isInIntakeZone(ArmZone zone) {
  //   return zone == ArmZone.CORAL_INTAKE || zone == ArmZone.ALGAE_INTAKE;
  // }

  // /** see Assets\Docs\TopUpperLimit.png */
  // public static ArmZone getArmZone(Rotation2d position) {
  //   double deg = position.getDegrees();
  //   if (deg >= Arm.TOP_LOWER_LIMIT.getDegrees() && deg <= Arm.TOP_UPPER_LIMIT.getDegrees()) {
  //     return ArmZone.TOP_ZONE;
  //   } else if (deg > Arm.TOP_UPPER_LIMIT.getDegrees()
  //       && deg < Arm.BOTTOM_LOWER_LIMIT.getDegrees()) {
  //     return ArmZone.CORAL_INTAKE;
  //   } else if (deg > Arm.BOTTOM_LOWER_LIMIT.getDegrees()
  //       && deg < Arm.BOTTOM_UPPER_LIMIT.getDegrees()) {
  //     return ArmZone.BOTTOM_ZONE;
  //   } else {
  //     return ArmZone.ALGAE_INTAKE;
  //   }
  // }

  public boolean superstructureInTolerance(SuperstructureState goal) {
    return arm.inTolerance(goal) && elevator.inTolerance(goal);
  }

  private Command setSingleState(SuperstructureState goal) {
    return Commands.run(
            () -> {
              desiredTransitionState = goal;
              arm.setState(goal.armState);
              elevator.setState(goal.elevatorState);
            },
            arm,
            elevator)
        .until(() -> superstructureInTolerance(goal))
        .finallyDo(
            () -> {
              currentState = goal;
            });
  }
}
