package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmIOFalcon implements ArmIO {

  private TalonFX motor;
  private CANcoder encoder;
  private MotionMagicExpoTorqueCurrentFOC angleRequest;
  public static final InvertedValue ARM_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;  //TODO find this
  public static final double ARM_ROTOR_TO_SENSOR_RATIO = 1; //TODO find this 
  public static final double MM_ACCELERATION = 0;
  public static final double MM_CRUISE_VELOCITY = 0;
  public static final double MM_EXPO_KA = 0;
  public static final double MM_EXPO_KV = 0;
  public static final double MM_JERK = 0;

  public ArmIOFalcon(int falconID, int encoderID) {
    
    motor = new TalonFX(falconID);
    TalonFXConfiguration angleConfigs = new TalonFXConfiguration();
    motor.getConfigurator().apply(angleConfigs);
    angleConfigs.MotorOutput.Inverted = ARM_MOTOR_INVERTED;
    angleConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleConfigs.MotionMagic.MotionMagicAcceleration = 0.0;
    angleConfigs.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    angleConfigs.MotionMagic.MotionMagicExpo_kA = 0.0;
    angleConfigs.MotionMagic.MotionMagicExpo_kV = 0;
    angleConfigs.MotionMagic.MotionMagicJerk = 0;
    angleConfigs.Feedback.FeedbackRemoteSensorID = encoderID;
    angleConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    angleConfigs.Feedback.RotorToSensorRatio = ARM_ROTOR_TO_SENSOR_RATIO;
    //angleConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true; TODO do we want this?
    //angleConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ShooterConstants.topLimit / 360;



    encoder = new CANcoder(encoderID);
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoder.getConfigurator().apply(encoderConfig);
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = 0.0; //TODO find this
    encoder.getConfigurator().apply(encoderConfig);
  
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.position = Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public void setState(ArmState state){
    double deg = state.degrees;
    
  }
}
