package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Superstructure.Superstructure;

public class ArmIOTalonFX implements ArmIO {
  public static final InvertedValue ARM_MOTOR_INVERTED =
      InvertedValue.CounterClockwise_Positive; // TODO find this
  public static final double ARM_ROTOR_TO_SENSOR_RATIO = 1; // TODO find this
  public static final double MM_CRUISE_VELOCITY = 0; // TODO find this
  public static final double MM_EXPO_KA = 0; // TODO find this
  public static final double MM_EXPO_KV = 0; // TODO find this
  public static final Rotation2d ARM_MIN_ANGLE = Rotation2d.fromDegrees(-180);
  public static final Rotation2d ARM_MAX_ANGLE = Rotation2d.fromDegrees(180);
  // how far away to stay from the intake
  // TODO tune this to be as small as safely possible
  public static final Rotation2d ANGLE_BUFFER = Rotation2d.fromDegrees(10);

  private TalonFX motor;
  private CANcoder encoder;
  private MotionMagicExpoTorqueCurrentFOC angleRequest;

  public ArmIOTalonFX(int falconID, int encoderID) {

    motor = new TalonFX(falconID);
    TalonFXConfiguration angleConfigs = new TalonFXConfiguration();
    motor.getConfigurator().apply(angleConfigs);
    angleConfigs.MotorOutput.Inverted = ARM_MOTOR_INVERTED;
    angleConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleConfigs.MotionMagic.MotionMagicCruiseVelocity = MM_CRUISE_VELOCITY;
    angleConfigs.MotionMagic.MotionMagicExpo_kA = MM_EXPO_KA;
    angleConfigs.MotionMagic.MotionMagicExpo_kV = MM_EXPO_KV;

    angleConfigs.Slot0.kG = 0; //TODO find these
    angleConfigs.Slot0.kS = 0;
    angleConfigs.Slot0.kA = 0;
    angleConfigs.Slot0.kP = 0;
    angleConfigs.Slot0.kD = 0;

    angleConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    angleConfigs.Feedback.FeedbackRemoteSensorID = encoderID;
    angleConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    angleConfigs.Feedback.RotorToSensorRatio = ARM_ROTOR_TO_SENSOR_RATIO;
    angleConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    angleConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ARM_MAX_ANGLE.getRotations();
    angleConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    angleConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ARM_MIN_ANGLE.getRotations();
    motor.getConfigurator().apply(angleConfigs);

    encoder = new CANcoder(encoderID);
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoder.getConfigurator().apply(encoderConfig);
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = 0.0; // TODO find this
    encoder.getConfigurator().apply(encoderConfig);

    angleRequest = new MotionMagicExpoTorqueCurrentFOC(0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // (-0.5, 0.5) rotations
    inputs.absolutePosition =
        Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble());
    // technically (-infinity, infinity),
    // however, the arm can only rotate from (-1,1) rotations //TODO check with electrical once this
    // is wired
    inputs.position = getPosition();
    inputs.zone = getArmZone();
  }

  private ArmZone getArmZone() {
    return Superstructure.getArmZone(getPosition());
  }

  private Rotation2d getPosition() {
    return Rotation2d.fromRotations(encoder.getPosition().getValueAsDouble());
  }

  /**
   * should be called every cycle, so that the arm collision avoidance gets updated
   *
   * @param state the desired state
   */
  @Override
  public void setDesiredState(Rotation2d rot) {
    motor.setControl(angleRequest.withPosition(rot.getRotations()));
  }

  
}

