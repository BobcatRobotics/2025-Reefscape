package frc.robot.subsystems.Superstructure.Elevator;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {

  public static final double GEAR_RATIO = // TODO i think this is wrong
      1936.0 / 192; // GEAR_RATIO motor rotations = 1 rotation of the ouput shaft
  public static final InvertedValue ELEVATOR_MOTOR_INVERTED =
      InvertedValue.CounterClockwise_Positive;

  /**
   * note that kG is different from ks, even they are both static forces, ks always opposes the
   * direction of motion, kg is always in the same direction, regardless of which way the elevator
   * is moving
   */
  private TalonFX motor;

  private CANcoder encoder;

  private MotionMagicExpoVoltage positionRequest =
      new MotionMagicExpoVoltage(0).withEnableFOC(false);
  private DutyCycleOut percentOutputRequest = new DutyCycleOut(0);

  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Angle> rotationalPosition;
  private StatusSignal<ControlModeValue> controlMode;
  private StatusSignal<Double> closedLoopReferenceSlope;
  private StatusSignal<Double> closedLoopReference;
  private StatusSignal<Current> supplyCurrent;
  private StatusSignal<Voltage> appliedVoltage;

  private ElevatorState desiredState = ElevatorState.UNKNOWN;

  private boolean isOverridden = false;

  public ElevatorIOTalonFX(int motorID, int encoderID) {
    motor = new TalonFX(motorID);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motor.getConfigurator().apply(motorConfig); // reset to factory default

    motorConfig.MotorOutput.Inverted = ELEVATOR_MOTOR_INVERTED;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 120;

    motorConfig.Slot0.kP = 3;
    motorConfig.Slot0.kI = 0;
    motorConfig.Slot0.kD = 0;
    motorConfig.Slot0.kS = 0.2;
    motorConfig.Slot0.kG = 0.9;
    motorConfig.Slot0.kA = 0.1;
    motorConfig.Slot0.kV = 1.25;
    motorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 0; // Expo mode, use max possible velocity;
    motorConfig.MotionMagic.MotionMagicExpo_kA = 0.7;
    motorConfig.MotionMagic.MotionMagicExpo_kV = 0.7;

    motorConfig.Voltage.PeakForwardVoltage =
        13; // this defaults to 16, battery supplies a max of 13, so idk why
    motorConfig.Voltage.PeakReverseVoltage = -13;

    motorConfig.Feedback.FeedbackRemoteSensorID = encoderID;
    motorConfig.Feedback.FeedbackSensorSource =
        FeedbackSensorSourceValue
            .RemoteCANcoder; // this should be fused but theres something funky with the gear ratio
    // rn
    motorConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Elevator.MAX_ROTATIONS.getRotations();
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    motor.getConfigurator().apply(motorConfig);

    encoder = new CANcoder(encoderID);
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoder.getConfigurator().apply(encoderConfig);
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = -0.45361328125; // TODO this keeps changing?
    encoder.getConfigurator().apply(encoderConfig);

    velocity = motor.getVelocity();
    rotationalPosition = encoder.getPosition();
    controlMode = motor.getControlMode();
    closedLoopReferenceSlope = motor.getClosedLoopReferenceSlope();
    closedLoopReference = motor.getClosedLoopReference();
    supplyCurrent = motor.getStatorCurrent();
    appliedVoltage = motor.getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Hertz.of(50),
        controlMode,
        velocity,
        rotationalPosition,
        closedLoopReference,
        closedLoopReferenceSlope,
        supplyCurrent,
        appliedVoltage);

    motor.optimizeBusUtilization();
    encoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocity,
        rotationalPosition,
        controlMode,
        closedLoopReferenceSlope,
        closedLoopReference,
        supplyCurrent,
        appliedVoltage);

    inputs.velocityRotPerSec = velocity.getValueAsDouble();
    inputs.rotPosition = Rotation2d.fromRotations(rotationalPosition.getValueAsDouble());
    inputs.positionPercent =
        rotationalPosition.getValueAsDouble() / Elevator.MAX_ROTATIONS.getRotations();
    inputs.aligned =
        Math.abs(rotationalPosition.getValueAsDouble() - desiredState.pos.getRotations())
            < Elevator.ELEVATOR_TOLERANCE.getRotations();
    inputs.motorConnected = motor.isConnected();
    inputs.encoderConnected = encoder.isConnected();
    inputs.controlMode = motor.getControlMode().getValue();
    inputs.positionRotations = inputs.rotPosition.getRotations();
    inputs.state = desiredState;
    inputs.heightMeters = inputs.positionRotations * Elevator.METERS_PER_ROTATION;
    inputs.distanceToAlignment =
        rotationalPosition.getValueAsDouble() - desiredState.pos.getRotations() * 360;
    inputs.overriden = isOverridden;
    inputs.closedLoopReference = closedLoopReference.getValueAsDouble();
    inputs.closedLoopReferenceSlope = closedLoopReferenceSlope.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
  }

  @Override
  public void setDesiredState(ElevatorState state) {
    this.desiredState = state;
    motor.setControl(positionRequest.withPosition(state.pos.getRotations()));
    isOverridden = false;
  }

  @Override
  public void manualOverride(double percent) {
    MathUtil.clamp(percent, -1, 1);
    percent = MathUtil.applyDeadband(percent, 0.05);
    motor.setControl(percentOutputRequest.withOutput(percent));
    isOverridden = true;
  }
}
