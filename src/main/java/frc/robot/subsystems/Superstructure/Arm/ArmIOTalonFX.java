package frc.robot.subsystems.Superstructure.Arm;

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
import org.littletonrobotics.junction.Logger;

public class ArmIOTalonFX implements ArmIO {
  public static final double ARM_ROTOR_TO_SENSOR_RATIO = 76.62;
  public static final Rotation2d ARM_MIN_ANGLE = Rotation2d.fromDegrees(-90);
  public static final Rotation2d ARM_MAX_ANGLE = Rotation2d.fromDegrees(270);

  private TalonFX motor;
  private CANcoder encoder;
  // TODO verify that you only have to set enableFOC once
  private MotionMagicExpoVoltage angleRequest = new MotionMagicExpoVoltage(0).withEnableFOC(true);
  private DutyCycleOut manualRequest = new DutyCycleOut(0);

  private StatusSignal<Angle> position;
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<ControlModeValue> controlMode;
  private StatusSignal<Double> closedLoopReferenceSlope;
  private StatusSignal<Double> closedLoopReference;
  private StatusSignal<Voltage> appliedVoltage;
  private StatusSignal<Current> appliedCurrent;

  private ArmState desiredState = ArmState.UNKOWN;
  private boolean flipped = false;
  private boolean isOverridden = false;

  public ArmIOTalonFX(int falconID, int encoderID) {

    motor = new TalonFX(falconID);
    TalonFXConfiguration angleConfigs = new TalonFXConfiguration();
    motor.getConfigurator().apply(angleConfigs);
    angleConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    angleConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    angleConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; // TODO find these
    angleConfigs.MotionMagic.MotionMagicExpo_kV = 0.12;

    // coral
    angleConfigs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    angleConfigs.Slot0.kP = 5;
    angleConfigs.Slot0.kI = 0;
    angleConfigs.Slot0.kD = 0;
    angleConfigs.Slot0.kS = 0.21;
    angleConfigs.Slot0.kG = 0.3;
    angleConfigs.Slot0.kA = 0.6;
    angleConfigs.Slot0.kV = 13;
    angleConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // algae
    angleConfigs.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    angleConfigs.Slot1.kP = 10;
    angleConfigs.Slot1.kI = 0;
    angleConfigs.Slot1.kD = 0;
    angleConfigs.Slot1.kS = 0.21;
    angleConfigs.Slot1.kG = 0.2;
    angleConfigs.Slot1.kA = 0;
    angleConfigs.Slot1.kV = 10;
    angleConfigs.Slot1.GravityType = GravityTypeValue.Arm_Cosine;

    // empty
    angleConfigs.Slot2.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    angleConfigs.Slot2.kP = 10;
    angleConfigs.Slot2.kI = 0;
    angleConfigs.Slot2.kD = 0;
    angleConfigs.Slot2.kS = 0.21;
    angleConfigs.Slot2.kG = 0.2;
    angleConfigs.Slot2.kA = 0;
    angleConfigs.Slot2.kV = 10;
    angleConfigs.Slot2.GravityType = GravityTypeValue.Arm_Cosine;

    angleConfigs.MotionMagic.MotionMagicCruiseVelocity = 0;
    angleConfigs.MotionMagic.MotionMagicExpo_kA = 2.5;
    angleConfigs.MotionMagic.MotionMagicExpo_kV = 3;

    angleConfigs.Voltage.PeakForwardVoltage = 13;
    angleConfigs.Voltage.PeakReverseVoltage = -13;

    angleConfigs.Feedback.FeedbackRemoteSensorID = encoderID;
    angleConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    angleConfigs.Feedback.RotorToSensorRatio = ARM_ROTOR_TO_SENSOR_RATIO;
    angleConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    angleConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ARM_MAX_ANGLE.getRotations();
    angleConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    angleConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ARM_MIN_ANGLE.getRotations();
    angleConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleConfigs.CurrentLimits.StatorCurrentLimit = 100;
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
    velocity = motor.getVelocity();
    // motion magic target velocity
    closedLoopReferenceSlope = motor.getClosedLoopReferenceSlope();
    // motion magic target position
    closedLoopReference = motor.getClosedLoopReference();
    appliedVoltage = motor.getMotorVoltage();
    appliedCurrent = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Hertz.of(50),
        controlMode,
        position,
        velocity,
        closedLoopReferenceSlope,
        closedLoopReference,
        appliedVoltage,
        appliedCurrent);

    encoder.optimizeBusUtilization();
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        position,
        velocity,
        controlMode,
        closedLoopReferenceSlope,
        closedLoopReference,
        appliedVoltage,
        appliedCurrent);

    // (-0.5, 0.5) rotations
    inputs.absolutePosition = Rotation2d.fromRotations(position.getValueAsDouble());
    inputs.encoderConnected = encoder.isConnected();
    inputs.motorConnected = motor.isConnected();
    inputs.state = desiredState;
    inputs.aligned =
        flipped
            ? Math.abs(position.getValueAsDouble() - desiredState.rotations - 1)
                < Arm.ARM_TOLERANCE.getRotations()
            : Math.abs(position.getValueAsDouble() - desiredState.rotations)
                < Arm.ARM_TOLERANCE.getRotations();
    inputs.velocityRotPerSec = velocity.getValueAsDouble();
    inputs.controlMode = motor.getControlMode().getValue();
    inputs.velocityRotPerSec = velocity.getValueAsDouble();
    inputs.positionDegrees = inputs.absolutePosition.getDegrees();
    inputs.desiredPositionDegrees = inputs.state.degrees;
    inputs.flipped = flipped;
    double rotations = flipped ? 0.5 - desiredState.rotations : desiredState.rotations;
    inputs.distanceToAlignment = Math.abs(position.getValueAsDouble() - rotations) * 360;
    inputs.isOverridden = isOverridden;
    inputs.closedLoopReferenceSlope = closedLoopReferenceSlope.getValueAsDouble();
    inputs.positionReference = closedLoopReference.getValueAsDouble();
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.appliedCurrent = appliedCurrent.getValueAsDouble();
  }

  /**
   * should be called every cycle, so that the arm collision avoidance gets updated
   *
   * @param state the desired state
   */
  @Override
  public void setDesiredState(ArmState state, boolean flipped, boolean hasPiece) {

    this.flipped = flipped;
    desiredState = state;
    double rotations = flipped ? 0.5 - state.rotations : state.rotations;

    int slot = 0; // coral

    if (hasPiece) {
      if ((state == ArmState.NET_SCORE || state == ArmState.NET_PREP)) {
        slot = 1; // algae
      }
    } else {
      slot = 2; // empty
    }

    motor.setControl(angleRequest.withPosition(rotations).withSlot(slot));
    Logger.recordOutput("arm slot", slot);
    isOverridden = false;
  }

  @Override
  public void manualOverride(double percent) {
    MathUtil.clamp(percent, -1, 1);
    percent = MathUtil.applyDeadband(percent, 0.05);
    motor.setControl(manualRequest.withOutput(percent));

    isOverridden = true;
  }
}
