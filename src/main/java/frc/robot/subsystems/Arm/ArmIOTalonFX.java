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
import frc.robot.subsystems.AlgaeIntake.AlgaeIntake;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.Superstructure.StateObserver;

public class ArmIOTalonFX implements ArmIO {
  public static final InvertedValue ARM_MOTOR_INVERTED =
      InvertedValue.CounterClockwise_Positive; // TODO find this
  public static final double ARM_ROTOR_TO_SENSOR_RATIO = 1; // TODO find this
  public static final double MM_CRUISE_VELOCITY = 0; // TODO find this
  public static final double MM_EXPO_KA = 0; // TODO find this
  public static final double MM_EXPO_KV = 0; // TODO find this
  public static final Rotation2d ARM_MIN_ANGLE = Rotation2d.fromDegrees(-360); // TODO find this
  public static final Rotation2d ARM_MAX_ANGLE = Rotation2d.fromDegrees(360); // TODO find this
  // how far away to stay from the intake
  // TODO tune this to be as small as safely possible
  public static final Rotation2d ANGLE_BUFFER = Rotation2d.fromDegrees(10);

  private TalonFX motor;
  private CANcoder encoder;
  private StateObserver observer;
  private MotionMagicExpoTorqueCurrentFOC angleRequest;

  public ArmIOTalonFX(int falconID, int encoderID) {
    this.observer = StateObserver.getInstance();

    motor = new TalonFX(falconID);
    TalonFXConfiguration angleConfigs = new TalonFXConfiguration();
    motor.getConfigurator().apply(angleConfigs);
    angleConfigs.MotorOutput.Inverted = ARM_MOTOR_INVERTED;
    angleConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleConfigs.MotionMagic.MotionMagicCruiseVelocity = MM_CRUISE_VELOCITY;
    angleConfigs.MotionMagic.MotionMagicExpo_kA = MM_EXPO_KA;
    angleConfigs.MotionMagic.MotionMagicExpo_kV = MM_EXPO_KV;

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
    return Arm.getArmZone(getPosition());
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
  public void setDesiredState(ArmState state) {
    // NO_OP = No Operation, do nothing
    if (!(state == ArmState.IDLE_NO_PIECE)) {

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

        double elevatorIntakeDelta =
            // if we're rotating ccw, then we'll going through the coral intake, //TODO check this
            // once architecture is finalized
            // so to get the height between the elevator and the intake,
            // we take the difference between the intake height and the elevator height
            optimalAngle.isCCW
                ? CoralIntake.CORAL_INTAKE_HEIGHT.baseUnitMagnitude() - elevatorHeight
                : AlgaeIntake.ALGAE_INTAKE_HEIGHT.baseUnitMagnitude() - elevatorHeight;

        // angle between elevator at its current height and intake
        Rotation2d theta =
            Rotation2d.fromRadians(
                Math.asin(elevatorIntakeDelta / Arm.LENGTH_TO_END_EFFECTOR.baseUnitMagnitude()));

        // this is the max angle the arm can go to without hitting the intake
        Rotation2d maxSafeAngle =
            optimalAngle.isCCW
                ? Rotation2d.kPi.minus(theta).minus(ANGLE_BUFFER)
                : theta.plus(ANGLE_BUFFER);

        motor.setControl(angleRequest.withPosition(maxSafeAngle.getRotations()));
      }
      // if we arent switching zones,
      // or we are clear of the intake,
      // or we are going from an intake zone to the top
      // set arm to the given state
      // assumes that if you are already in a zone it is safe to travel
      // within it
      else if (state.zone == getArmZone()
          || observer.elevatorAboveIntakeMinimum()
          || (observer.isInIntakeZone() && state.zone == ArmZone.TOP_ZONE)) {
        motor.setControl(angleRequest.withPosition(optimalAngle.angle.getRotations()));
      }
    }
  }

  /**
   * Returns the best angle to go to, given a desired angle (in [0,360)) and an arbitrary allowed
   * range [minAngle, maxAngle].
   *
   * <p>The arm’s allowed range may be, for example, [-270,270] or [-180,180]. The desired angle is
   * given as a Rotation2d (which should represent an angle in [0,360)).
   *
   * <p>The algorithm adjusts the desired angle by adding or subtracting multiples of 360° so that
   * the target is as close as possible to the current position (i.e. the difference is no more than
   * 180°) and ensures that the result is within the allowed range.
   *
   * <p>If both paths (clockwise or counterclockwise) are valid and equal in magnitude, the
   * counterclockwise path is chosen.
   *
   * @param desiredAngle the desired angle (interpreted modulo 360°)
   * @param minAngle the minimum allowed angle (in degrees)
   * @param maxAngle the maximum allowed angle (in degrees)
   * @return an ArmMovement containing the target angle (as a Rotation2d) and the direction
   */
  public ArmMovement optimalAngle(Rotation2d desiredAngle) {
    // current position in degrees (assuming encoder returns revolutions that are then scaled)
    double current = encoder.getPosition().getValueAsDouble() * 360;

    // normalize the desired angle to [0, 360)
    double candidate = desiredAngle.getDegrees() % 360;
    if (candidate < 0) {
      candidate += 360;
    }

    // Adjust candidate by adding/subtracting multiples of 360 so that
    // the difference (candidate - current) lies in [-180, 180]
    while (candidate - current > 180) {
      candidate -= 360;
    }
    while (candidate - current < -180) {
      candidate += 360;
    }

    // Now ensure that candidate is within the allowed range.
    // If it is outside, adjust by a full rotation until it is.
    while (candidate > ARM_MAX_ANGLE.getDegrees()) {
      candidate -= 360;
    }
    while (candidate < ARM_MIN_ANGLE.getDegrees()) {
      candidate += 360;
    }

    // Compute the rotation direction: if the delta is positive, move CCW.
    double delta = candidate - current;
    boolean isCCW = delta > 0;

    return new ArmMovement(isCCW, Rotation2d.fromDegrees(candidate));
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
