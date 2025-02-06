package frc.robot.subsystems.StateMachine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class StateGraph {
    private final Map<SuperstructureState, List<SuperstructureState>> adjacencyList = new HashMap<>();

    // Add a state to the graph
    public void addState(SuperstructureState state) {
        adjacencyList.putIfAbsent(state, new ArrayList<>());
    }

    // Add a valid transition between two states
    public void addTransition(SuperstructureState from, SuperstructureState to) {
        adjacencyList.get(from).add(to);
    }

    // Get neighbors (valid next states)
    public List<SuperstructureState> getNeighbors(SuperstructureState state) {
        return adjacencyList.getOrDefault(state, new ArrayList<>());
    }

      /**
   * creates a java representation of the graph of all posible transitions
   * outlined here:
   * {@linkplain https://www.chiefdelphi.com/t/team-177-429-bobcat-robotics-program-2025-build-blog/478233/62}
   * 
   */
  public static StateGraph initializeGraph() {
    // initialize empty graph
    StateGraph graph = new StateGraph();

    // Add all states to the graph
    // this creates the nodes (potential states),
    // we then add the edges (valid transitions between states) later
    for (SuperstructureState state : SuperstructureState.values()) {
      graph.addState(state);
    }

    //create list of all possible transitions
    //first element in list is the parent, the following elements
    //are all the children, or possible transitions from the parent state
    List<SuperstructureState[]> transitions = List.of(
      new SuperstructureState[]{SuperstructureState.IDLE_NO_PIECE, SuperstructureState.CORAL_HANDOFF, SuperstructureState.INTAKE_ALGAE_GROUND, SuperstructureState.ALGAE_PREP_L2, SuperstructureState.ALGAE_PREP_L3},
      new SuperstructureState[]{SuperstructureState.CORAL_HANDOFF, SuperstructureState.IDLE_NO_PIECE, SuperstructureState.ELEVATOR_SAFE_ZONE},
      new SuperstructureState[]{SuperstructureState.ELEVATOR_SAFE_ZONE, SuperstructureState.IDLE_CORAL},
      new SuperstructureState[]{SuperstructureState.IDLE_CORAL, SuperstructureState.CORAL_PREP_L1, SuperstructureState.CORAL_PREP_L2, SuperstructureState.CORAL_PREP_L3, SuperstructureState.CORAL_PREP_L4}, //TODO add outtake
      new SuperstructureState[]{SuperstructureState.CORAL_PREP_L1, SuperstructureState.CORAL_PREP_L2, SuperstructureState.CORAL_PREP_L3, SuperstructureState.CORAL_PREP_L4, SuperstructureState.CORAL_SCORE_L1, SuperstructureState.IDLE_NO_PIECE, SuperstructureState.IDLE_CORAL},
      new SuperstructureState[]{SuperstructureState.CORAL_PREP_L2, SuperstructureState.CORAL_PREP_L1, SuperstructureState.CORAL_PREP_L3, SuperstructureState.CORAL_PREP_L4, SuperstructureState.CORAL_SCORE_L2, SuperstructureState.IDLE_NO_PIECE, SuperstructureState.IDLE_CORAL},
      new SuperstructureState[]{SuperstructureState.CORAL_PREP_L3, SuperstructureState.CORAL_PREP_L1, SuperstructureState.CORAL_PREP_L2, SuperstructureState.CORAL_PREP_L4, SuperstructureState.CORAL_SCORE_L3, SuperstructureState.IDLE_NO_PIECE, SuperstructureState.IDLE_CORAL},
      new SuperstructureState[]{SuperstructureState.CORAL_PREP_L4, SuperstructureState.CORAL_PREP_L1,SuperstructureState.CORAL_PREP_L2, SuperstructureState.CORAL_PREP_L3, SuperstructureState.CORAL_SCORE_L4, SuperstructureState.IDLE_NO_PIECE, SuperstructureState.IDLE_CORAL},
      new SuperstructureState[]{SuperstructureState.CORAL_SCORE_L1, SuperstructureState.CORAL_PREP_L1},
      new SuperstructureState[]{SuperstructureState.CORAL_SCORE_L2, SuperstructureState.CORAL_PREP_L2},
      new SuperstructureState[]{SuperstructureState.CORAL_SCORE_L3, SuperstructureState.CORAL_PREP_L3},
      new SuperstructureState[]{SuperstructureState.CORAL_SCORE_L4, SuperstructureState.CORAL_PREP_L4},
      new SuperstructureState[]{SuperstructureState.IDLE_ALGAE, SuperstructureState.ALGAE_SCORE_PROCESSOR}, //TODO add outtake
      new SuperstructureState[]{SuperstructureState.ALGAE_SCORE_PROCESSOR, SuperstructureState.ELEVATOR_SAFE_ZONE},
      new SuperstructureState[]{SuperstructureState.INTAKE_ALGAE_GROUND, SuperstructureState.IDLE_ALGAE, SuperstructureState.IDLE_NO_PIECE},
      new SuperstructureState[]{SuperstructureState.ALGAE_PREP_L2, SuperstructureState.IDLE_ALGAE, SuperstructureState.ALGAE_SCORE_L2, SuperstructureState.ALGAE_PREP_L3},
      new SuperstructureState[]{SuperstructureState.ALGAE_PREP_L3, SuperstructureState.IDLE_ALGAE, SuperstructureState.ALGAE_SCORE_L3, SuperstructureState.ALGAE_PREP_L2},
      new SuperstructureState[]{SuperstructureState.ALGAE_SCORE_L2, SuperstructureState.ALGAE_SCORE_L2},
      new SuperstructureState[]{SuperstructureState.ALGAE_SCORE_L3, SuperstructureState.ALGAE_SCORE_L3},
      new SuperstructureState[]{SuperstructureState.ELEVATOR_SAFE_ZONE, SuperstructureState.IDLE_CORAL, SuperstructureState.IDLE_NO_PIECE}
    );

    //for each set of transitions
    for (SuperstructureState[] transition : transitions){
      SuperstructureState parent = transition[0];
      //for each transition in the set
      for(int i = 1; i<transition.length; i++){
        //add the edge to the graph
        graph.addTransition(parent, transition[i]);
      }
    }
    
    return graph;
  }

    
}
