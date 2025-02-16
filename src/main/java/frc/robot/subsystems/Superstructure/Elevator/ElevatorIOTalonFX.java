package frc.robot.subsystems.Superstructure.Elevator;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  private MotionMagicExpoTorqueCurrentFOC positionRequest = new MotionMagicExpoTorqueCurrentFOC(0);
  private VoltageOut voltageRequest = new VoltageOut(0);

  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Current> torqueCurrent;
  private StatusSignal<Angle> rotationalPosition;

  private ElevatorState desiredState;

  public ElevatorIOTalonFX(int motorID, int encoderID) {
    motor = new TalonFX(motorID);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motor.getConfigurator().apply(motorConfig); // reset to factory default

    motorConfig.MotorOutput.Inverted = ELEVATOR_MOTOR_INVERTED;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 0; // unlimited
    // 63.972168; //TODO theoretical value
    motorConfig.MotionMagic.MotionMagicExpo_kA = 0;
    motorConfig.MotionMagic.MotionMagicExpo_kV = 0;

    // torque current, dont use kv and ka?
    motorConfig.Slot0.kG = 0; // TODO find this
    motorConfig.Slot0.kS = 0; // TODO find this
    motorConfig.Slot0.kP = 0.273285; // TODO theoretical value

    motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    motorConfig.Feedback.FeedbackRemoteSensorID = encoderID;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1; // max 360 deg ccw
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -1; // max -360 deg cw

    torqueCurrent = motor.getTorqueCurrent();
    velocity = motor.getVelocity();
    rotationalPosition = encoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Hertz.of(50), torqueCurrent, velocity, rotationalPosition);

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
        rotationalPosition.getValueAsDouble() / Elevator.MAX_ROTATIONS.getRotations();
    inputs.aligned =
        Math.abs(rotationalPosition.getValueAsDouble() - desiredState.pos.getRotations())
            < Elevator.ELEVATOR_TOLERANCE.getRotations();
    inputs.motorConnected = motor.isConnected();
    inputs.encoderConnected = encoder.isConnected();
    inputs.controlMode = motor.getControlMode().getValue();
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
