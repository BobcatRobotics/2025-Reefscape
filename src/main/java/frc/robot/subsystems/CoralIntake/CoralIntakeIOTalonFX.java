package frc.robot.subsystems.CoralIntake;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class CoralIntakeIOTalonFX implements CoralIntakeIO {

  private TalonFX roller;
  private TalonFX pivot;
  private TalonFX carwash;
  private PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0);
  private VelocityTorqueCurrentFOC rollerRequest = new VelocityTorqueCurrentFOC(0);
  private VelocityTorqueCurrentFOC carwashRequest = new VelocityTorqueCurrentFOC(0);

  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Angle> position;

  private IntakeState currState = IntakeState.UNKNOWN;

  public CoralIntakeIOTalonFX(int rollerMotorID, int pivotMotorID, int carwashID) {
    roller = new TalonFX(rollerMotorID);
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    roller.getConfigurator().apply(rollerConfig);
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 80;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO check
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.Slot0.kS = 1; // TODO tune
    rollerConfig.Slot0.kP = 1; // // TODO tune
    roller.getConfigurator().apply(rollerConfig);

    pivot = new TalonFX(pivotMotorID);
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    roller.getConfigurator().apply(pivotConfig);
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 60;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO check
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // TODO tune
    pivotConfig.Slot0.kG = 1; // TODO tune
    pivotConfig.Slot0.kP = 1; // TODO tune
    roller.getConfigurator().apply(pivotConfig);
    pivot = new TalonFX(pivotMotorID);


    carwash = new TalonFX(carwashID);
    TalonFXConfiguration carwashConfig = new TalonFXConfiguration();
    carwash.getConfigurator().apply(carwashConfig);
    carwashConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    carwashConfig.CurrentLimits.StatorCurrentLimit = 60;
    carwashConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO check
    carwashConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    carwashConfig.Slot0.kP = 1; // TODO tune
    carwash.getConfigurator().apply(carwashConfig);

    velocity = roller.getVelocity();
    position = pivot.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), velocity, position);

    pivot.optimizeBusUtilization();
    roller.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(CoralIntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, position);
    inputs.intakeVelocity = velocity.getValue();
    inputs.position = position.getValue();
    inputs.pivotMotorConnected = pivot.isConnected();
    inputs.rollerMotorConnected = roller.isConnected();
    inputs.carwashMotorConnected = carwash.isConnected();
    inputs.state = currState;
  }

  @Override
  public void setSpeed(AngularVelocity velocity) {
    roller.setControl(rollerRequest.withVelocity(velocity));
    carwash.setControl(carwashRequest.withVelocity(velocity)); //TODO this is kinda lazy, we should probably set it seperately
  };

  @Override
  public void setAngle(Angle angle) {
    pivot.setControl(positionRequest.withPosition(angle));
    currState = IntakeState.UNKNOWN;
  }

  @Override
  public void setState(IntakeState state) {
    pivot.setControl(positionRequest.withPosition(state.angle));
    currState = state;
  }

  @Override
  public void retract() {
    setState(IntakeState.RETRACT);
  }

  @Override
  public void deploy() {
    setState(IntakeState.DEPLOY);
  }

  @Override
  public void stop(){
    setSpeed(DegreesPerSecond.of(0));
  }
}

// fun fact: A group of Anands is called a department
