package frc.robot.subsystems.CoralIntake;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.TunerConstants25;

public class CoralIntakeIOTalonFX implements CoralIntakeIO {

  private TalonFX roller;
  private TalonFX pivot;
  private TalonFX carwash;
  private LaserCan laser;
  private PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0);
  private VelocityVoltage rollerRequest = new VelocityVoltage(0);
  private VelocityVoltage carwashRequest = new VelocityVoltage(0);
  private Alert rangingAlert =
      new Alert(
          "Couldnt set intake ranging mode!! OMG this is really bad!! the robot will EXPLODE!!!!!! fix IMEDIATELY or i'll DIE a GRUESOME and PAINFUL death!",
          AlertType.kWarning);

  private StatusSignal<AngularVelocity> rollerVelocity;
  private StatusSignal<AngularVelocity> carwashVelocity;
  // private StatusSignal<Angle> position;

  private AngularVelocity desiredCarwashVelocity = RotationsPerSecond.of(0);
  private AngularVelocity desiredRollerVelocity = RotationsPerSecond.of(0);

  private IntakeState currState = IntakeState.UNKNOWN;

  public CoralIntakeIOTalonFX(int rollerMotorID, int pivotMotorID, int carwashID, int laserCANID) {

    // laser = new LaserCan(laserCANID);

    // try {
    // laser.setRangingMode(RangingMode.SHORT);
    // rangingAlert.set(false);
    // } catch (ConfigurationFailedException e) {
    // e.printStackTrace();
    // rangingAlert.set(true);
    // }

    roller = new TalonFX(rollerMotorID, TunerConstants25.kCANBus);
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    roller.getConfigurator().apply(rollerConfig);
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 80;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO check
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.Slot0.kP = 0.4; // // TODO tune
    roller.getConfigurator().apply(rollerConfig);

    pivot = new TalonFX(pivotMotorID, TunerConstants25.kCANBus);
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    roller.getConfigurator().apply(pivotConfig);
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 60;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO check
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // TODO tune
    pivotConfig.Slot0.kG = 1; // TODO tune
    pivotConfig.Slot0.kP = 1; // TODO tune
    roller.getConfigurator().apply(pivotConfig);

    carwash = new TalonFX(carwashID);
    TalonFXConfiguration carwashConfig = new TalonFXConfiguration();
    carwash.getConfigurator().apply(carwashConfig);
    carwashConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    carwashConfig.CurrentLimits.StatorCurrentLimit = 60;
    carwashConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO check
    carwashConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    carwashConfig.Slot0.kP = 0.4; // TODO tune
    carwash.getConfigurator().apply(carwashConfig);

    rollerVelocity = roller.getVelocity();
    carwashVelocity = carwash.getVelocity();
    // position = pivot.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), rollerVelocity, carwashVelocity);

    pivot.optimizeBusUtilization();
    roller.optimizeBusUtilization();
    carwash.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(CoralIntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(rollerVelocity, carwashVelocity);
    inputs.intakeVelocity = rollerVelocity.getValue();
    // inputs.position = position.getValue();
    inputs.pivotMotorConnected = pivot.isConnected();
    inputs.rollerMotorConnected = roller.isConnected();
    inputs.carwashMotorConnected = carwash.isConnected();
    inputs.state = currState;
    inputs.carwashVelocity = carwashVelocity.getValue();
    inputs.desiredCarwashVelocity = desiredCarwashVelocity;
    inputs.desiredRollerVelocity = desiredRollerVelocity;
  }

  @Override
  public void setSpeed(AngularVelocity velocity) {
    roller.setControl(rollerRequest.withVelocity(velocity));
    carwash.setControl(
        carwashRequest.withVelocity(
            velocity)); // TODO this is kinda lazy, we should probably set it seperately

    desiredCarwashVelocity = velocity;
    desiredRollerVelocity = velocity;
  }

  @Override
  public void setAngle(Angle angle) {
    pivot.setControl(positionRequest.withPosition(outputAngleToMotorRotations(angle)));
    currState = IntakeState.UNKNOWN;
  }

  @Override
  public void setState(IntakeState state) {
    pivot.setControl(positionRequest.withPosition(outputAngleToMotorRotations(state.angle)));
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
  public void stop() {
    roller.stopMotor();
    carwash.stopMotor();
  }

  private double outputAngleToMotorRotations(Angle pivotAngle) {
    return pivotAngle.in(Rotations) * CoralIntake.GEAR_RATIO;
  }
}

// fun fact: A group of Anands is called a department
