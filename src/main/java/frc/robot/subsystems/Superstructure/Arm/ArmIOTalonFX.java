package frc.robot.subsystems.Superstructure.Arm;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
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

public class ArmIOTalonFX implements ArmIO {
  public static final double ARM_ROTOR_TO_SENSOR_RATIO = 76.62;
  public static final Rotation2d ARM_MIN_ANGLE = Rotation2d.fromDegrees(-90);
  public static final Rotation2d ARM_MAX_ANGLE = Rotation2d.fromDegrees(270);

  private TalonFX motor;
  private CANcoder encoder;
  private MotionMagicTorqueCurrentFOC angleRequest = new MotionMagicTorqueCurrentFOC(0);

  private StatusSignal<Angle> position;
  private StatusSignal<Current> torqueCurrentAmps;
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<ControlModeValue> controlMode;

  private ArmState desiredState = ArmState.UNKOWN;

  public ArmIOTalonFX(int falconID, int encoderID) {

    motor = new TalonFX(falconID);
    TalonFXConfiguration angleConfigs = new TalonFXConfiguration();
    motor.getConfigurator().apply(angleConfigs);
    angleConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    angleConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    angleConfigs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    angleConfigs.Slot0.kP = 25;
    angleConfigs.Slot0.kI = 0;
    angleConfigs.Slot0.kD = 10;
    angleConfigs.Slot0.kS = 5;
    angleConfigs.Slot0.kG = 6;
    angleConfigs.MotionMagic.MotionMagicCruiseVelocity = 2.5;
    angleConfigs.MotionMagic.MotionMagicAcceleration = 1;

    angleConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    angleConfigs.Feedback.FeedbackRemoteSensorID = encoderID;
    angleConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    angleConfigs.Feedback.RotorToSensorRatio = ARM_ROTOR_TO_SENSOR_RATIO;
    angleConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    angleConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ARM_MAX_ANGLE.getRotations();
    angleConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    angleConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ARM_MIN_ANGLE.getRotations();
    angleConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleConfigs.CurrentLimits.StatorCurrentLimit = 60;
    angleConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motor.getConfigurator().apply(angleConfigs);

    encoder = new CANcoder(encoderID);
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoder.getConfigurator().apply(encoderConfig);
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = 0.2;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    encoder.getConfigurator().apply(encoderConfig);

    controlMode = motor.getControlMode();
    position = encoder.getAbsolutePosition();
    torqueCurrentAmps = motor.getTorqueCurrent();
    velocity = motor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Hertz.of(50), controlMode, position, torqueCurrentAmps, velocity);

    encoder.optimizeBusUtilization();
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(position);
    // (-0.5, 0.5) rotations
    inputs.absolutePosition = Rotation2d.fromRotations(position.getValueAsDouble());
    inputs.encoderConnected = encoder.isConnected();
    inputs.motorConnected = motor.isConnected();
    inputs.state = desiredState;
    inputs.aligned =
        Math.abs(position.getValueAsDouble() - desiredState.rotations)
            < Arm.ARM_TOLERANCE.getRotations();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.velocityRotPerSec = velocity.getValueAsDouble();
    inputs.controlMode = motor.getControlMode().getValue();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.velocityRotPerSec = velocity.getValueAsDouble();
    inputs.positionDegrees = inputs.absolutePosition.getDegrees();
    inputs.desiredPositionRotation = inputs.state.rotations;
    inputs.distanceToAlignment =
        Math.abs(position.getValueAsDouble() - desiredState.rotations) * 360;
    // inputs.zone = getArmZone();
  }

  // private ArmZone getArmZone() {
  // return Superstructure.getArmZone(getPosition());
  // }

  /**
   * should be called every cycle, so that the arm collision avoidance gets updated
   *
   * @param state the desired state
   */
  @Override
  public void setDesiredState(ArmState state) {
    desiredState = state;
    motor.setControl(angleRequest.withPosition(state.rotations));
  }
}
