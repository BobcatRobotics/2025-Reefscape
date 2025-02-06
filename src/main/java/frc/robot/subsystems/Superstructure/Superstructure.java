package frc.robot.subsystems.Superstructure;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Elevator.Elevator;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Set;

public class Superstructure {
  private SuperstructureState currentState = SuperstructureState.IDLE_NO_PIECE;
  private Arm arm;
  private Elevator elevator;
  Alert stateAlert =
      new Alert("Attempted to set superstructure to invalid state!", AlertType.kError);
  private StateGraph graph;

  /** A class representing the Arm-Elevator superstructure */
  public Superstructure(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;
    stateAlert.set(false);
    graph = StateGraph.initializeGraph();
  }

  /**
   * @param desiredState the state we want to set the robot to
   * @return {@code true} if the state was set, {@code false} if the state was invalid
   */
  public boolean setState(SuperstructureState desiredState) {
    // make sure state is valid
    // will the arm hit the intake or floor of the robot in its final state?
    if (Elevator.checkForArmCollision(desiredState.armState.zone, desiredState.elevatorState)) {
      stateAlert.set(true);
      return false; // dont set the arm to the invalid state
    }
    ;

    currentState = desiredState;

    elevator.setState(desiredState.elevatorState);
    arm.setState(desiredState.armState);
    return true;
  }

  public SuperstructureState getState() {
    return currentState;
  }

  /**
   * @param goal the desired state
   * @return whether or not the elevator will have to move before the arm swings fully down
   */
  public boolean isTransitionSafe(SuperstructureState goal) {
    // make sure state is valid
    // will the arm hit the intake or floor of the robot in its final state?
    if (Elevator.checkForArmCollision(goal.armState.zone, goal.elevatorState)) {
      return false;
    }
    ;

    // if the arm swings down before the elevator raises, will it hit?
    if (Elevator.checkForArmCollision(goal.armState.zone, currentState.elevatorState)) {
      return false;
    }
    return true;
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
}
