package frc.robot.subsystems.Superstructure;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class StateGraph {
  private static StateGraph instance;

  private static List<SuperstructureState> prepStates =
      Arrays.asList(
          new SuperstructureState[] {
            SuperstructureState.CORAL_PREP_L1,
            SuperstructureState.CORAL_PREP_L2,
            SuperstructureState.CORAL_PREP_L3,
            SuperstructureState.PRE_CORAL_PREP_L4,
            SuperstructureState.ALGAE_PREP_L2,
            SuperstructureState.ALGAE_PREP_L3,
            SuperstructureState.NET_PREP,
            SuperstructureState.RIGHT_SIDE_UP_IDLE
          });

  private final Map<SuperstructureState, List<SuperstructureState>> adjacencyList = new HashMap<>();

  public static StateGraph getInstance() {
    if (instance == null) {
      instance = initializeGraph();
    }
    return instance;
  }

  /**
   * Add a state to the graph, if it isnt already there
   *
   * @param state node to add
   */
  public void addState(SuperstructureState state) {
    adjacencyList.putIfAbsent(state, new ArrayList<>());
  }

  /**
   * adds a valid transition between two states to the graph note that this is NOT two directional
   *
   * @param from parent state
   * @param to child state
   */
  public void addTransition(SuperstructureState from, SuperstructureState to) {
    adjacencyList.get(from).add(to);
  }

  // Get neighbors (valid next states)
  public List<SuperstructureState> getNeighbors(SuperstructureState state) {
    return adjacencyList.getOrDefault(state, new ArrayList<>());
  }

  /**
   * creates a java representation of the graph of all posible transitions outlined here:
   * {@linkplain
   * https://www.chiefdelphi.com/t/team-177-429-bobcat-robotics-program-2025-build-blog/478233/62}
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

    // create list of all possible transitions
    // first element in list is the parent, the following elements
    // are all the children, or possible transitions from the parent state

    // IF TWO PATHS HAVE SAME LENGTH, FIRST ELEMENT IN LIST IS PRIORITIZED
    List<SuperstructureState[]> transitions =
        List.of(
            new SuperstructureState[] {
              SuperstructureState.UNKNOWN,
              SuperstructureState.ELEVATOR_SAFE_ZONE,
              SuperstructureState.CORAL_PREP_L1,
              SuperstructureState.CORAL_PREP_L2,
              SuperstructureState.CORAL_PREP_L3,
              SuperstructureState.PRE_CORAL_PREP_L4,
              SuperstructureState.NET_PREP
            },
            new SuperstructureState[] {
              SuperstructureState.HANDOFF_FLIP_SAFE_ZONE, SuperstructureState.UPSIDE_DOWN_IDLE
            },
            new SuperstructureState[] {
              SuperstructureState.UPSIDE_DOWN_IDLE,
              SuperstructureState.HANDOFF_PREP,
              SuperstructureState.ELEVATOR_AND_ARM_SAFE_ZONE,
              SuperstructureState.ALGAE_PREP_L2,
              SuperstructureState.ALGAE_PREP_L3,
              SuperstructureState.ELEVATOR_SAFE_ZONE,
              SuperstructureState.CLIMB,
              SuperstructureState.HUMAN_INTAKE,
            },
            new SuperstructureState[] {
              SuperstructureState.CORAL_HANDOFF,
              SuperstructureState.UPSIDE_DOWN_IDLE,
              SuperstructureState.ELEVATOR_SAFE_ZONE,
              SuperstructureState.HANDOFF_PREP
            },
            new SuperstructureState[] {
              SuperstructureState.HANDOFF_PREP,
              SuperstructureState.CORAL_HANDOFF,
              SuperstructureState.UPSIDE_DOWN_IDLE,
              SuperstructureState.ELEVATOR_SAFE_ZONE
            },
            new SuperstructureState[] {
              SuperstructureState.ELEVATOR_SAFE_ZONE,
              SuperstructureState.UPSIDE_DOWN_IDLE,
              SuperstructureState.RIGHT_SIDE_UP_IDLE,
              SuperstructureState.IDLE_ALGAE,
              SuperstructureState.CORAL_PREP_L1,
              SuperstructureState.CORAL_PREP_L2,
              SuperstructureState.CORAL_PREP_L3,
              SuperstructureState.CORAL_PREP_L4,
              SuperstructureState.PRE_CORAL_PREP_L4,
              SuperstructureState.ALGAE_PREP_L2,
              SuperstructureState.ALGAE_PREP_L3
            },
            new SuperstructureState[] {
              SuperstructureState.RIGHT_SIDE_UP_IDLE,
              SuperstructureState.HANDOFF_FLIP_SAFE_ZONE,
              SuperstructureState.NET_SCORE,
              SuperstructureState.NET_PREP,
              SuperstructureState.CORAL_PREP_L1,
              SuperstructureState.CORAL_PREP_L2,
              SuperstructureState.CORAL_PREP_L3,
              SuperstructureState.CORAL_PREP_L4,
              SuperstructureState.PRE_CORAL_PREP_L4,
              SuperstructureState.CORAL_PREP_L4,
              SuperstructureState.ALGAE_PREP_L2,
              SuperstructureState.ALGAE_PREP_L3,
              SuperstructureState.HUMAN_INTAKE,
              SuperstructureState.INTAKE_ALGAE_GROUND,
              SuperstructureState.CLIMB,
              SuperstructureState.POPSICLE_LICK,
              SuperstructureState.ALGAE_SCORE_PROCESSOR,
            },
            new SuperstructureState[] {
              SuperstructureState.HANDOFF_FLIP_SAFE_ZONE, SuperstructureState.ELEVATOR_SAFE_ZONE
            },
            new SuperstructureState[] {
              SuperstructureState.CORAL_PREP_L1,
              SuperstructureState.CORAL_PREP_L2,
              SuperstructureState.CORAL_PREP_L3,
              SuperstructureState.CORAL_PREP_L4,
              SuperstructureState.PRE_CORAL_PREP_L4,
              SuperstructureState.CORAL_SCORE_L1,
              SuperstructureState.RIGHT_SIDE_UP_IDLE,
              SuperstructureState.NET_SCORE,
              SuperstructureState.INTAKE_ALGAE_GROUND,
            },
            new SuperstructureState[] {
              SuperstructureState.CORAL_PREP_L2,
              SuperstructureState.CORAL_PREP_L1,
              SuperstructureState.CORAL_PREP_L3,
              SuperstructureState.CORAL_PREP_L4,
              SuperstructureState.PRE_CORAL_PREP_L4,
              SuperstructureState.CORAL_SCORE_L2,
              SuperstructureState.RIGHT_SIDE_UP_IDLE,
              SuperstructureState.NET_SCORE,
              SuperstructureState.INTAKE_ALGAE_GROUND,
            },
            new SuperstructureState[] {
              SuperstructureState.CORAL_PREP_L3,
              SuperstructureState.CORAL_PREP_L1,
              SuperstructureState.CORAL_PREP_L2,
              SuperstructureState.CORAL_PREP_L4,
              SuperstructureState.PRE_CORAL_PREP_L4,
              SuperstructureState.CORAL_SCORE_L3,
              SuperstructureState.RIGHT_SIDE_UP_IDLE,
              SuperstructureState.NET_SCORE,
              SuperstructureState.INTAKE_ALGAE_GROUND,
            },
            new SuperstructureState[] {
              SuperstructureState.PRE_CORAL_PREP_L4,
              SuperstructureState.CORAL_PREP_L1,
              SuperstructureState.CORAL_PREP_L2,
              SuperstructureState.CORAL_PREP_L3,
              SuperstructureState.CORAL_SCORE_L4,
              SuperstructureState.RIGHT_SIDE_UP_IDLE,
              SuperstructureState.INTAKE_ALGAE_GROUND,
              SuperstructureState.NET_SCORE,
              SuperstructureState.CORAL_PREP_L4,
            },
            new SuperstructureState[] {
              SuperstructureState.CORAL_PREP_L4,
              SuperstructureState.CORAL_SCORE_L4,
              SuperstructureState.PRE_CORAL_PREP_L4,
              SuperstructureState.POST_CORAL_SCORE_L4,
              SuperstructureState.CORAL_PREP_L1,
              SuperstructureState.CORAL_PREP_L2,
              SuperstructureState.CORAL_PREP_L3,
              SuperstructureState.INTAKE_ALGAE_GROUND,
            },
            new SuperstructureState[] {
              SuperstructureState.CORAL_SCORE_L1,
              SuperstructureState.CORAL_PREP_L1,
              SuperstructureState.RIGHT_SIDE_UP_IDLE,
              SuperstructureState.UPSIDE_DOWN_IDLE
            },
            new SuperstructureState[] {
              SuperstructureState.CORAL_SCORE_L2,
              SuperstructureState.POST_CORAL_SCORE_L2,
              SuperstructureState.CORAL_PREP_L2,
              SuperstructureState.CORAL_PREP_L1
            },
            new SuperstructureState[] {
              SuperstructureState.POST_CORAL_SCORE_L2,
              SuperstructureState.RIGHT_SIDE_UP_IDLE,
              SuperstructureState.ELEVATOR_SAFE_ZONE
            },
            new SuperstructureState[] {
              SuperstructureState.CORAL_SCORE_L3,
              SuperstructureState.POST_CORAL_SCORE_L3,
              SuperstructureState.CORAL_PREP_L3,
              SuperstructureState.CORAL_SCORE_L1
            },
            new SuperstructureState[] {
              SuperstructureState.POST_CORAL_SCORE_L3,
              SuperstructureState.RIGHT_SIDE_UP_IDLE,
              SuperstructureState.UPSIDE_DOWN_IDLE
            },
            new SuperstructureState[] {
              SuperstructureState.CORAL_SCORE_L4,
              SuperstructureState.POST_CORAL_SCORE_L4,
              SuperstructureState.PRE_CORAL_PREP_L4,
              SuperstructureState.CORAL_PREP_L4,
              SuperstructureState.RIGHT_SIDE_UP_IDLE
            },
            new SuperstructureState[] {
              SuperstructureState.POST_CORAL_SCORE_L4,
              SuperstructureState.RIGHT_SIDE_UP_IDLE,
              SuperstructureState.UPSIDE_DOWN_IDLE,
              SuperstructureState.CORAL_SCORE_L4,
              SuperstructureState.PRE_CORAL_PREP_L4,
              SuperstructureState.CORAL_PREP_L4,
              SuperstructureState.ALGAE_PREP_L2,
              SuperstructureState.ALGAE_PREP_L3,
            },
            new SuperstructureState[] {
              SuperstructureState.IDLE_ALGAE,
              SuperstructureState.ALGAE_SCORE_PROCESSOR,
              SuperstructureState.RIGHT_SIDE_UP_IDLE,
              SuperstructureState.INTAKE_ALGAE_GROUND,
              SuperstructureState.NET_PREP
            },
            new SuperstructureState[] {
              SuperstructureState.ALGAE_SCORE_PROCESSOR,
              SuperstructureState.RIGHT_SIDE_UP_IDLE,
              SuperstructureState.ELEVATOR_SAFE_ZONE,
              SuperstructureState.ELEVATOR_AND_ARM_SAFE_ZONE,
              SuperstructureState.INTAKE_ALGAE_GROUND
            },
            new SuperstructureState[] {
              SuperstructureState.INTAKE_ALGAE_GROUND,
              SuperstructureState.ALGAE_SCORE_PROCESSOR,
              SuperstructureState.ELEVATOR_SAFE_ZONE,
              SuperstructureState.ELEVATOR_AND_ARM_SAFE_ZONE,
              SuperstructureState.IDLE_ALGAE,
              SuperstructureState.RIGHT_SIDE_UP_IDLE
            },
            new SuperstructureState[] {
              SuperstructureState.ELEVATOR_AND_ARM_SAFE_ZONE, SuperstructureState.RIGHT_SIDE_UP_IDLE
            },
            new SuperstructureState[] {
              SuperstructureState.ALGAE_PREP_L2,
              SuperstructureState.NET_PREP,
              SuperstructureState.IDLE_ALGAE,
              SuperstructureState.RIGHT_SIDE_UP_IDLE,
              SuperstructureState.ALGAE_PREP_L3,
              SuperstructureState.CORAL_PREP_L1,
              SuperstructureState.CORAL_PREP_L2,
              SuperstructureState.CORAL_PREP_L3,
              SuperstructureState.PRE_CORAL_PREP_L4,
              SuperstructureState.UPSIDE_DOWN_IDLE
            },
            new SuperstructureState[] {
              SuperstructureState.ALGAE_PREP_L3,
              SuperstructureState.NET_PREP,
              SuperstructureState.IDLE_ALGAE,
              SuperstructureState.RIGHT_SIDE_UP_IDLE,
              SuperstructureState.ALGAE_PREP_L2,
              SuperstructureState.CORAL_PREP_L1,
              SuperstructureState.CORAL_PREP_L2,
              SuperstructureState.CORAL_PREP_L3,
              SuperstructureState.PRE_CORAL_PREP_L4,
              SuperstructureState.UPSIDE_DOWN_IDLE
            },
            new SuperstructureState[] {
              SuperstructureState.NET_SCORE,
              SuperstructureState.NET_PREP,
              SuperstructureState.CORAL_PREP_L1,
              SuperstructureState.CORAL_PREP_L2,
              SuperstructureState.CORAL_PREP_L3,
              SuperstructureState.PRE_CORAL_PREP_L4,
              SuperstructureState.RIGHT_SIDE_UP_IDLE
            },
            new SuperstructureState[] {
              SuperstructureState.HUMAN_INTAKE, SuperstructureState.POST_HUMAN_INTAKE
            },
            new SuperstructureState[] {
              SuperstructureState.POST_HUMAN_INTAKE,
              SuperstructureState.RIGHT_SIDE_UP_IDLE,
              SuperstructureState.ELEVATOR_SAFE_ZONE
            },
            new SuperstructureState[] {
              SuperstructureState.CLIMB, SuperstructureState.RIGHT_SIDE_UP_IDLE
            },
            new SuperstructureState[] {
              SuperstructureState.NET_PREP,
              SuperstructureState.NET_SCORE,
              SuperstructureState.RIGHT_SIDE_UP_IDLE
            },
            new SuperstructureState[] {
              SuperstructureState.POPSICLE_LICK,
              SuperstructureState.RIGHT_SIDE_UP_IDLE,
              SuperstructureState.CORAL_PREP_L1,
              SuperstructureState.CORAL_PREP_L2,
              SuperstructureState.CORAL_PREP_L3,
              SuperstructureState.PRE_CORAL_PREP_L4
            });

    // for each set of transitions
    for (SuperstructureState[] transition : transitions) {
      SuperstructureState parent = transition[0];
      // for each transition in the set
      for (int i = 1; i < transition.length; i++) {
        // add the edge to the graph
        graph.addTransition(parent, transition[i]);
      }
    }

    return graph;
  }
}

// Fun fact: Akilan Anand and Adithya Anand are brothers
