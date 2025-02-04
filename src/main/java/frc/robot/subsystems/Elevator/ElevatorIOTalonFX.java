package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOTalonFX implements ElevatorIO {
  
  public static final InvertedValue ELEVATOR_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
  public static final double ELEVATOR_ROTOR_TO_SENSOR_RATIO = 1;

  public static final double MM_ACCELERATION = 0;
  public static final double MM_CRUISE_VELOCITY = 0;
  public static final double MM_EXPO_KA = 0;
  public static final double MM_EXPO_KV = 0;
  public static final double MM_JERK = 0;

  /**
   * note that kG is different from ks, even they are both static forces, ks always opposes the direction of motion,
   * kg is always in the same direction, regardless of which way the elevator is moving
   */
  public static final double kG = 0;
  public static final double kS = 0;
  public static final double kV = 0;
  public static final double kA = 0;
  public static final double kP = 0;
  public static final double kD = 0;


  
  private TalonFX motor;
  private CANcoder encoder;
  private MotionMagicExpoTorqueCurrentFOC positionRequest = new MotionMagicExpoTorqueCurrentFOC(0);

  public ElevatorIOTalonFX(int motorID, int encoderID) {
    motor = new TalonFX(motorID);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motor.getConfigurator().apply(motorConfig); //reset to factory default

    motorConfig.MotorOutput.Inverted = ELEVATOR_MOTOR_INVERTED;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = MM_CRUISE_VELOCITY;
    motorConfig.MotionMagic.MotionMagicExpo_kA = MM_EXPO_KA;
    motorConfig.MotionMagic.MotionMagicExpo_kV = MM_EXPO_KV;

    motorConfig.Slot0.kG = 0;
    motorConfig.Slot0.kS = 0;
    motorConfig.Slot0.kA = 0;
    motorConfig.Slot0.kP = 0;
    motorConfig.Slot0.kD = 0;

    motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    motorConfig.Feedback.FeedbackRemoteSensorID = encoderID;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfig.Feedback.RotorToSensorRatio = ELEVATOR_ROTOR_TO_SENSOR_RATIO;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1; // max 360 deg ccw
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -1; // max -360 deg cw
  }
}
