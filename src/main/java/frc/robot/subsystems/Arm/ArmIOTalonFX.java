package frc.robot.subsystems.Arm;

import java.lang.Thread.State;

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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntake;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.StateMachine.StateObserver;

public class ArmIOTalonFX implements ArmIO {
  private MotionMagicExpoTorqueCurrentFOC angleRequest;
  public static final InvertedValue ARM_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive; // TODO find this
  public static final double ARM_ROTOR_TO_SENSOR_RATIO = 1; // TODO find this
  public static final double MM_ACCELERATION = 0;
  public static final double MM_CRUISE_VELOCITY = 0;
  public static final double MM_EXPO_KA = 0;
  public static final double MM_EXPO_KV = 0;
  public static final double MM_JERK = 0;
  // how far away to stay from the intake
  // TODO tune this to be as small as safely possible
  public static final Rotation2d ANGLE_BUFFER = Rotation2d.fromDegrees(10);
  // How much buffer to give when avoiding the intake
  public static final Rotation2d ALIGNMENT_TOLERANCE = Rotation2d.fromDegrees(10);

  private TalonFX motor;
  private CANcoder encoder;
  private StateObserver observer;

  private Alert stateRequiresElevatorTravel = new Alert(
      "Arm waiting for elevator movement",
      AlertType.kInfo);

  public ArmIOTalonFX(int falconID, int encoderID, StateObserver observer) {
    this.observer = observer;

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



    angleConfigs.Slot0.kG = 0;
    angleConfigs.Slot0.kS = 0;
    angleConfigs.Slot0.kA = 0;
    angleConfigs.Slot0.kP = 0;
    angleConfigs.Slot0.kD = 0;

    angleConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    angleConfigs.Feedback.FeedbackRemoteSensorID = encoderID;
    angleConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    angleConfigs.Feedback.RotorToSensorRatio = ARM_ROTOR_TO_SENSOR_RATIO;
    angleConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    angleConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1; // max 360 deg ccw
    angleConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    angleConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -1; // max -360 deg cw

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
    inputs.absolutePosition = Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble());
    // technically (-infinity, infinity),
    // however, the arm can only rotate from (-1,1) rotations
    inputs.position = getPosition();
    inputs.zone = getArmZone();
  }

  private ArmZone getArmZone() {
    return Arm.getArmZone(getPosition());
  }

  private Rotation2d getPosition() {
    return Rotation2d.fromRotations(encoder.getPosition().getValueAsDouble());
  }

  /**
   * should be called every cycle, so that the arm collision avoidance gets
   * updated
   * 
   * @param state the desired state
   */
  @Override
  public void setDesiredState(ArmState state) {
    // NO_OP = No Operation, do nothing
    if (!(state == ArmState.NO_OP)) {

      // calculate the optimal path to the desired angle
      // this will give us the shortest path to the desired position
      // that doesnt overextend
      ArmMovement optimalAngle = optimalAngle(state.rot2d);

      // height between the top of the intake and the bottom elevator
      double elevatorHeight = observer.elevatorHeight.baseUnitMagnitude();

      // if we want to go to the bottom, then we have to wait for
      // the elevator to raise so we dont hit the intake
      // we will set the arm as low as it can go (with a buffer)
      // without hitting the intake in the meantime
      // this makes it so that by the time the elevator is high enough
      // the arm only has to travel a small distance

      // note that the more we raise the elevator, the more we can
      // lower the arm, which is why we have to call this every cycle,
      // so that the arm setpoint gets updated

      // if we want to go to the bottom zone and we arent already in it,
      // or we want to go to one of the intake zones
      // and the elevator isnt past its clearance point yet
      // go as far down as possible without hitting the intake

      if (((state.zone == ArmZone.BOTTOM_ZONE && getArmZone() != ArmZone.BOTTOM_ZONE)
          || (StateObserver.isInIntakeZone(state.zone)))
          && !observer.elevatorAboveIntakeMinimum()) {
        stateRequiresElevatorTravel.set(true);

        double elevatorIntakeDelta =
            // if we're rotating ccw, then we'll going through the coral intake, //TODO check this once architecture is finalized
            // so to get the height between the elevator and the intake,
            // we take the difference between the intake height and the elevator height
            optimalAngle.isCCW ? CoralIntake.CORAL_INTAKE_HEIGHT.baseUnitMagnitude() - elevatorHeight
                : AlgaeIntake.ALGAE_INTAKE_HEIGHT.baseUnitMagnitude() - elevatorHeight;

        // angle between elevator at its current height and intake
        Rotation2d theta = Rotation2d
            .fromRadians(
                Math.asin(elevatorIntakeDelta / Arm.LENGTH_TO_END_EFFECTOR.baseUnitMagnitude()));

        // this is the max angle the arm can go to without hitting the intake
        Rotation2d maxSafeAngle = optimalAngle.isCCW ? Rotation2d.kPi.minus(theta).minus(ANGLE_BUFFER) 
            : theta.plus(ANGLE_BUFFER);

        motor.setControl(angleRequest.withPosition(maxSafeAngle.getRotations()));
      }
      // if we arent switching zones, 
      //or we are clear of the intake,
      //or we are going from an intake zone to the top
      //set arm to the given state
      //assumes that if you are already in a zone it is safe to travel
      //within it
      else if (state.zone == getArmZone()
          || observer.elevatorAboveIntakeMinimum()
          || (observer.isInIntakeZone() && state.zone==ArmZone.TOP_ZONE)) {
        motor.setControl(angleRequest.withPosition(optimalAngle.angle.getRotations()));
      }
    }
  }

  /**
   * <p>
   * Returns the best angle to go to given a desired angle within (0, 360)
   * 
   * <p>
   * the arm has a range of (-360,360), we need a way to calculate the shortest
   * angle to go to without overextending.
   * 
   * <p>
   * if both paths are valid and equal length, the ccw path will be returned
   * (i.e. 90 -> 270 can either go to -90 or 270, 270 will be returned)
   * 
   * <p>
   * for example:
   * if current = 360 and desired = 10
   * <p>
   * output is 10
   * 
   * <p>
   * if current = -360 and desired = 10
   * <p>
   * output is -350
   * 
   * <p>
   * if current = -270 and desired = 90
   * <p>
   * output is 0
   * 
   * 
   * @param desiredAngle
   * @return
   */
  public ArmMovement optimalAngle(Rotation2d desiredAngle) {

    // lowkey this is confusing as heck
    // i wrote this at 3am on 400mg of caffine and i don't remember how it works
    // but it passes every test I run on it ¯\_(ツ)_/¯

    double current = encoder.getPosition().getValueAsDouble() * 360;
    // normalize the desired angle to [0, 360)
    double candidate = desiredAngle.getDegrees() % 360;
    if (candidate < 0) {
      candidate += 360;
    }

    // adjust candidate to be as close as possible to 'current'
    // (i.e. the difference between candidate and current is no more than 180
    // degrees)
    while (candidate - current > 180) {
      candidate -= 360;
    }
    while (candidate - current < -180) {
      candidate += 360;
    }

    // ensure candidate is within the valid range [-360, 360].
    // (this might be necessary if, for example, candidate became 370 or -370)
    if (candidate > 360) {
      candidate -= 360;
    } else if (candidate < -360) {
      candidate += 360;
    }

    // Calculate the difference to determine rotation direction
    double delta = candidate - current;
    boolean isCCW = delta > 0;

    return new ArmMovement(isCCW, desiredAngle);
  }

}

class ArmMovement {
  public Rotation2d angle;
  public boolean isCCW;

  public ArmMovement(boolean isCCW, Rotation2d angle) {
    this.isCCW = isCCW;
    this.angle = angle;
  }
}
