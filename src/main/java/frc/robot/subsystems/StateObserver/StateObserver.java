package frc.robot.subsystems.StateObserver;

public class StateObserver {
    public StateObserver instance;
    public SuperstructureState currentState;
    public SuperstructureState lastState;

    public StateObserver(){}
    
    public StateObserver getInstance(){
        if(instance == null){
            instance = new StateObserver();
        }
        return instance;
    }

    public void setSuperstructureState(SuperstructureState desiredState){
        lastState = currentState;
        currentState = desiredState;
    }


}

enum SuperstructureState{

}

