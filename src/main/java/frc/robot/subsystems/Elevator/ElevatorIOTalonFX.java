package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
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

public class ElevatorIOTalonFX implements ElevatorIO {

  public static final double GEAR_RATIO = 10.08 / 1; // GEAR_RATIO motor rotations = 1 rotation of the ouput shaft TODO
                                                     // FIND THIS
  public static final InvertedValue ELEVATOR_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive; //TODO find this

  public static final double MM_ACCELERATION = 0; // TODO find this
  public static final double MM_CRUISE_VELOCITY = 0; // TODO find this
  public static final double MM_EXPO_KA = 0; // TODO find this
  public static final double MM_EXPO_KV = 0; // TODO find this
  public static final double MM_JERK = 0; // TODO find this

  /**
   * note that kG is different from ks, even they are both static forces, ks
   * always opposes the
   * direction of motion, kg is always in the same direction, regardless of which
   * way the elevator
   * is moving
   */
  public static final double kG = 0;

  public static final double kS = 0; // TODO tune
  public static final double kV = 0; // TODO tune
  public static final double kA = 0; // TODO tune
  public static final double kP = 0; // TODO tune
  public static final double kD = 0; // TODO tune

  private TalonFX motor;
  private CANcoder encoder;
  private MotionMagicExpoTorqueCurrentFOC positionRequest = new MotionMagicExpoTorqueCurrentFOC(0);

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
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = MM_CRUISE_VELOCITY;
    motorConfig.MotionMagic.MotionMagicExpo_kA = MM_EXPO_KA;
    motorConfig.MotionMagic.MotionMagicExpo_kV = MM_EXPO_KV;

    motorConfig.Slot0.kG = kG;
    motorConfig.Slot0.kS = kS;
    motorConfig.Slot0.kA = kA;
    motorConfig.Slot0.kP = kP;
    motorConfig.Slot0.kD = kD;

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

    BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50),
        torqueCurrent, velocity, rotationalPosition);

    motor.optimizeBusUtilization();
    encoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(torqueCurrent, velocity, rotationalPosition);
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.velocityRotPerSec = velocity.getValueAsDouble();
    inputs.rotPosition = Rotation2d.fromRotations(
        rotationalPosition.getValueAsDouble());
    inputs.positionPercent = rotationalPosition.getValueAsDouble() / Elevator.MAX_ROTATIONS.getRotations();
    inputs.aligned =
    Math.abs(
      rotationalPosition.getValueAsDouble() - desiredState.pos.getRotations()) 
      < Elevator.ELEVATOR_TOLERANCE.getRotations();
  }



  @Override
  public void setDesiredState(ElevatorState state) {
    this.desiredState = state;
    motor.setControl(
        positionRequest.withPosition(
            state.pos.getRotations()
      ));
  }


}
