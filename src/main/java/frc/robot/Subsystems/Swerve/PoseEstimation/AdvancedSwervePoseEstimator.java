package frc.robot.Subsystems.Swerve.PoseEstimation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * This class wraps {@link SwerveDriveOdometry Swerve Drive Odometry} to fuse latency-compensated
 * vision measurements with swerve drive encoder distance measurements. It is intended to be a
 * drop-in replacement for {@link SwerveDriveOdometry}.
 *
 * <p>{@link AdvancedSwervePoseEstimator#update} should be called every robot loop.
 *
 * <p>{@link AdvancedSwervePoseEstimator#addVisionMeasurement} can be called as infrequently as you
 * want; if you never call it, then this class will behave as regular encoder odometry.
 */
public class AdvancedSwervePoseEstimator extends AdvancedPoseEstimator<SwerveModulePosition[]> {
  private final int m_numModules;

  /**
   * Constructs a SwerveDrivePoseEstimator.
   *
   * @param kinematics A correctly-configured kinematics object for your drivetrain.
   * @param gyroAngle The current gyro angle.
   * @param modulePositions The current distance and rotation measurements of the swerve modules.
   * @param initialPoseMeters The starting pose estimate.
   * @param stateStdDevs Standard deviations of the pose estimate (x position in meters, y position
   *     in meters, and heading in radians). Increase these numbers to trust your state estimate
   *     less.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less.
   */
  public AdvancedSwervePoseEstimator(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPoseMeters,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N3, N1> visionMeasurementStdDevs) { // TODO optional pure odometry tracker
    super(
        kinematics,
        new SwerveDriveOdometry(kinematics, gyroAngle, modulePositions, initialPoseMeters),
        stateStdDevs,
        visionMeasurementStdDevs);

    m_numModules = modulePositions.length;
  }

  @Override
  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
    if (wheelPositions.length != m_numModules) {
      throw new IllegalArgumentException(
          "Number of modules is not consistent with number of wheel locations provided in "
              + "constructor");
    }

    return super.updateWithTime(currentTimeSeconds, gyroAngle, wheelPositions);
  }

  /** updates odometry, with new std devs */
  public Pose2d updateWithTime(
      double currentTimeSeconds,
      Rotation2d gyroAngle,
      SwerveModulePosition[] wheelPositions,
      Matrix<N3, N1> stdDevs) {
    if (wheelPositions.length != m_numModules) {
      throw new IllegalArgumentException(
          "Number of modules is not consistent with number of wheel locations provided in "
              + "constructor");
    }
    super.setStateMeasurementStdDevs(stdDevs);
    return super.updateWithTime(currentTimeSeconds, gyroAngle, wheelPositions);
  }

  /** updates odometry, with new std devs */
  public Pose2d update(
      Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Matrix<N3, N1> stdDevs) {
    if (wheelPositions.length != m_numModules) {
      throw new IllegalArgumentException(
          "Number of modules is not consistent with number of wheel locations provided in "
              + "constructor");
    }

    super.setStateMeasurementStdDevs(stdDevs);
    return super.update(gyroAngle, wheelPositions);
  }
}
