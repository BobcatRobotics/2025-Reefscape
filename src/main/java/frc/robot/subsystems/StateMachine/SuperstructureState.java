package frc.robot.subsystems.StateMachine;


public class SuperstructureState {
    private ArmState armState;
    private IntakeState intakeState;
    private EndEffectorState endEffectorState;

    public SuperstructureState(ArmState armState, IntakeState intakeState, EndEffectorState endEffectorState) {
        this.armState = armState;
        this.intakeState = intakeState;
        this.endEffectorState = endEffectorState;
    }
    public SuperstructureState(ArmState armState, IntakeState intakeState) {
        this.armState = armState;
        this.intakeState = intakeState;
        this.endEffectorState = EndEffectorState.NO_OP;
    }
    public SuperstructureState(ArmState armState, EndEffectorState endEffectorState) {
        this.armState = armState;
        this.intakeState = IntakeState.NO_OP;
        this.endEffectorState = endEffectorState;
    }
    public SuperstructureState(IntakeState intakeState, EndEffectorState endEffectorState) {
        this.armState = ArmState.NO_OP;
        this.intakeState = intakeState;
        this.endEffectorState = endEffectorState;
    }
    public SuperstructureState(ArmState armState) {
        this.armState = armState;
        this.intakeState = IntakeState.NO_OP;
        this.endEffectorState = EndEffectorState.NO_OP;
    }
    public SuperstructureState(IntakeState intakeState) {
        this.armState = ArmState.NO_OP;
        this.intakeState = intakeState;
        this.endEffectorState = EndEffectorState.NO_OP;
    }
    public SuperstructureState(EndEffectorState endEffectorState) {
        this.armState = ArmState.NO_OP;
        this.intakeState = IntakeState.NO_OP;
        this.endEffectorState = endEffectorState;
    }
    public SuperstructureState() {
        this.armState = ArmState.NO_OP;
        this.intakeState = IntakeState.NO_OP;
        this.endEffectorState = EndEffectorState.NO_OP;
    }

     

    public ArmState getArmState() {
        return armState;
    }
    public IntakeState getIntakeState() {
        return intakeState;
    }
    public EndEffectorState getEndEffectorState() {
        return endEffectorState;
    }

    public static SuperstructureState IDLE = new SuperstructureState(ArmState.IDLE, IntakeState.IDLE, EndEffectorState.IDLE);
    public static SuperstructureState INTAKE = new SuperstructureState(ArmState.INTAKE, IntakeState.INTAKE, EndEffectorState.NO_OP);
}

enum ArmState {
    IDLE,
    INTAKE,
    SCOREL1,
    SCOREL2,
    SCOREL3,
    SCOREL4,
    LOW_AlGAE,
    HIGH_AlGAE,
    PICKUP,
    NO_OP
}

enum EndEffectorState {
    IDLE,
    PICKUP,
    OUTTAKE,
    NO_OP
}

enum IntakeState {
    IDLE,
    INTAKE,
    OUTTAKE,
    NO_OP
}
