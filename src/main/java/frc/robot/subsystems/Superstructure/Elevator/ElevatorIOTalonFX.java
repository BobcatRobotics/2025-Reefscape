package frc.robot.subsystems.Superstructure.Elevator;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {

  public static final double GEAR_RATIO =
      10.08 / 1; // GEAR_RATIO motor rotations = 1 rotation of the ouput shaft
  public static final InvertedValue ELEVATOR_MOTOR_INVERTED =
      InvertedValue.CounterClockwise_Positive; // TODO find this

  /**
   * note that kG is different from ks, even they are both static forces, ks always opposes the
   * direction of motion, kg is always in the same direction, regardless of which way the elevator
   * is moving
   */
  private TalonFX motor;

  private CANcoder encoder;

  private PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0);
  private VoltageOut voltageRequest = new VoltageOut(0);

  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Current> torqueCurrent;
  private StatusSignal<Angle> rotationalPosition;
  private StatusSignal<ControlModeValue> controlMode;

  private ElevatorState desiredState = ElevatorState.UNKNOWN;

  public ElevatorIOTalonFX(int motorID, int encoderID) {
    motor = new TalonFX(motorID);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motor.getConfigurator().apply(motorConfig); // reset to factory default

    motorConfig.MotorOutput.Inverted = ELEVATOR_MOTOR_INVERTED;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // TODO 120 stator limit

    // torque current, dont use kv and ka?
    motorConfig.Slot0.kP = 30;
    motorConfig.Slot0.kI = 25;
    motorConfig.Slot0.kD = 8;
    motorConfig.Slot0.kS = 20;
    motorConfig.Slot0.kG = 38;
    motorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    motorConfig.Feedback.FeedbackRemoteSensorID = encoderID;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Elevator.ELEVATOR_MAX_ROTATIONS.getRotations();
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    motor.getConfigurator().apply(motorConfig);

    encoder = new CANcoder(encoderID);
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoder.getConfigurator().apply(encoderConfig);
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = 0.219971; // TODO find this
    encoder.getConfigurator().apply(encoderConfig);

    torqueCurrent = motor.getTorqueCurrent();
    velocity = motor.getVelocity();
    rotationalPosition = encoder.getPosition();
    controlMode = motor.getControlMode();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Hertz.of(50), controlMode, torqueCurrent, velocity, rotationalPosition);

    motor.optimizeBusUtilization();
    encoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(torqueCurrent, velocity, rotationalPosition);
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.velocityRotPerSec = velocity.getValueAsDouble();
    inputs.rotPosition = Rotation2d.fromRotations(rotationalPosition.getValueAsDouble());
    inputs.positionPercent =
        rotationalPosition.getValueAsDouble() / Elevator.ELEVATOR_MAX_ROTATIONS.getRotations();
    inputs.aligned =
        Math.abs(rotationalPosition.getValueAsDouble() - desiredState.pos.getRotations())
            < Elevator.ELEVATOR_TOLERANCE.getRotations();
    inputs.motorConnected = motor.isConnected();
    inputs.encoderConnected = encoder.isConnected();
    inputs.controlMode = motor.getControlMode().getValue();
    inputs.positionRotations = inputs.rotPosition.getRotations();
    inputs.state = desiredState;
  }

  @Override
  public void setDesiredState(ElevatorState state) {
    this.desiredState = state;
    motor.setControl(positionRequest.withPosition(state.pos.getRotations()));
  }

  @Override
  public void runVoltage(Voltage volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }
}
