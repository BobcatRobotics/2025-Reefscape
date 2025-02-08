package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.ReefHeight;
import frc.robot.subsystems.Drive.Drive;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = .5;
  private static final double ANGLE_KD = 0;
  private static final double DRIVE_KPY = 1;
  private static final double DRIVE_KDY = 0;
  private static final double DRIVE_KPX = 1;
  private static final double DRIVE_KDX = 0;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final Distance ALIGN_DISTANCE = Meters.of(.4);

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command fieldRelativeJoystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  public static Command robotCentricJoystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          drive.runVelocity(speeds);
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAndAlign(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(
            () -> {
              angleController.reset(drive.getRotation().getRadians());
            });
  }

  public static Command singleTagAlign(
      Drive drive,
      DoubleSupplier distanceSupplier,
      DoubleSupplier horizontalSupplier,
      Supplier<Rotation2d> omegaSupplier) {

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    LinearFilter omegaFilter = LinearFilter.movingAverage(50);

    ProfiledPIDController distanceController =
        new ProfiledPIDController(
            DRIVE_KPX, 0.0, DRIVE_KDX, new TrapezoidProfile.Constraints(1, 1.0));
    LinearFilter xFilter = LinearFilter.movingAverage(50);

    ProfiledPIDController horizontalController =
        new ProfiledPIDController(
            DRIVE_KPY, 0.0, DRIVE_KDY, new TrapezoidProfile.Constraints(1, 1.0));
    LinearFilter yFilter = LinearFilter.movingAverage(50);

    return Commands.run(
            () -> {
              double distToTag = distanceSupplier.getAsDouble();
              double horizontalDistance = horizontalSupplier.getAsDouble();
              double omega = omegaSupplier.get().getRadians();
              double filteredDistance = 0;
              double filteredHorizontal = 0;
              double filteredOmega = 0;

              if (distToTag != 0) {
                filteredDistance = xFilter.calculate(distToTag);
                Logger.recordOutput("SingleTagAlign/filteredDistance", filteredDistance);
              }
              if (horizontalDistance != 0) {
                filteredHorizontal = yFilter.calculate(horizontalDistance);
                Logger.recordOutput("SingleTagAlign/filteredHorizontal", filteredHorizontal);
              }
              if (omega != 0) {
                filteredOmega = omegaFilter.calculate(omega);
                Logger.recordOutput("SingleTagAlign/filteredOmega", filteredOmega);
              }

              double omegaOutput =
                  filteredOmega == 0 ? 0 : angleController.calculate(filteredOmega, 0);

              double distanceOutput =
                  distToTag == 0 ? 0 : distanceController.calculate(filteredDistance, 2);

              double horizontalOutput =
                  horizontalDistance == 0
                      ? 0
                      : horizontalController.calculate(filteredHorizontal, 0);

              // Get linear velocity
              // Translation2d linearVelocity = new
              // Translation2d(distanceController.calculate(xf,
              // ANGLE_KD),2);

              // Apply rotation deadband
              // double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

              // Square rotation value for more precise control
              // omega = Math.copySign(omega * omega, omega);

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      distanceOutput * drive.getMaxLinearSpeedMetersPerSec(),
                      horizontalOutput * drive.getMaxLinearSpeedMetersPerSec(),
                      omegaOutput * drive.getMaxAngularSpeedRadPerSec());
              drive.runVelocity(speeds);
            },
            drive)
        .beforeStarting(
            () -> {
              angleController.reset(drive.getRotation().getRadians());
            });
  }

  public static Command alignToTag(
      Drive drive, Supplier<Rotation2d> tx, DoubleSupplier ty, DoubleSupplier distanceToTag) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    ProfiledPIDController xController =
        new ProfiledPIDController(
            DRIVE_KPX, 0.0, DRIVE_KDX, new TrapezoidProfile.Constraints(1, 1.0));
    LinearFilter xFilter = LinearFilter.movingAverage(50);

    ProfiledPIDController yController =
        new ProfiledPIDController(
            DRIVE_KPY, 0.0, DRIVE_KDY, new TrapezoidProfile.Constraints(1, 1.0));
    LinearFilter yFilter = LinearFilter.movingAverage(50);

    // Construct command
    return Commands.run(
        () -> {
          double distToTag = distanceToTag.getAsDouble();
          double yDist = ty.getAsDouble();
          double filteredDistance = 0;
          double filteredY = 0;

          if (distToTag != 0) {
            filteredDistance = xFilter.calculate(distToTag);
          }
          if (ty.getAsDouble() != 0) {
            filteredY = yFilter.calculate(yDist);
          }

          Logger.recordOutput("FilteredX", filteredDistance);
          Logger.recordOutput("FilteredY", filteredY);

          Logger.recordOutput("DistToTag", filteredDistance);
          // Calculate angular speed
          double omega =
              tx.get().getRadians() == 0
                  ? 0
                  : angleController.calculate(
                      drive.getRotation().getRadians(), tx.get().getRadians());

          double distanceOutput =
              distanceToTag.getAsDouble() == 0 ? 0 : xController.calculate(distToTag, 2);
          Logger.recordOutput("disttotagpid", distanceOutput);

          double yOutput = yController.calculate(yDist, 0);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  omega == 0
                      ? distanceOutput * (1 / omega)
                      : distanceOutput, // get to within 1 meter of the tag, output scales as
                  // angular error decreases\
                  yOutput,
                  0);
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);

    // Reset PID controller when command starts
    // .beforeStarting(
    // () -> {
    // angleController.reset(filteredDistance);
    // });
  }

  public static Command driveToReef(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier clockwiseSupplier,
      BooleanSupplier counterclockwiseSupplier) {

    List<Pose2d> faces = Arrays.asList(FieldConstants.Reef.centerFaces);

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    ProfiledPIDController xController =
        new ProfiledPIDController(
            DRIVE_KPX, 0.0, DRIVE_KDX, new TrapezoidProfile.Constraints(5, 3.0));

    ProfiledPIDController yController =
        new ProfiledPIDController(
            DRIVE_KPY, 0.0, DRIVE_KDY, new TrapezoidProfile.Constraints(5, 3.0));

    return Commands.run(
            () -> {
              Pose2d nearestFace = drive.getPose().nearest(faces);
              Logger.recordOutput("reef_face/raw", nearestFace);
              double adjustY = 0;

              if (clockwiseSupplier.getAsBoolean()) {
                adjustY = -FieldConstants.Reef.reefToBranchY;
            } else if (counterclockwiseSupplier.getAsBoolean()) {
                adjustY = FieldConstants.Reef.reefToBranchY;
            }

            //   List<Map<ReefHeight, Pose3d>> offsetPositions = FieldConstants.Reef.branchPositions;

              // double xOffset =
              // ALIGN_DISTANCE.baseUnitMagnitude()
              // * Math.cos(nearestFace.getRotation().getRadians());
              // double yOffset =
              // ALIGN_DISTANCE.baseUnitMagnitude()
              // * Math.sin(nearestFace.getRotation().getRadians());

              // Pose2d offsetFace =
              // nearestFace.plus(new Transform2d(xOffset, yOffset, new Rotation2d()));
              // Logger.recordOutput("reef_face/offset", offsetFace);
              int faceIndex = -1;
              for (int i = 0; i < FieldConstants.Reef.centerFaces.length; i++) {
                if (FieldConstants.Reef.centerFaces[i] == nearestFace) {
                  faceIndex = i;
                  break;
                }
              }


              Pose2d poseDirection =
                  new Pose2d(
                      FieldConstants.Reef.center, Rotation2d.fromDegrees(180 - (60 * faceIndex)));

              double adjustX =
                  ALIGN_DISTANCE.baseUnitMagnitude() + FieldConstants.Reef.faceToCenter;
            //   double adjustY = Units.inchesToMeters(0);

              Pose2d offsetFace =
                  new Pose2d(
                      new Translation2d(
                          poseDirection
                              .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                              .getX(),
                          poseDirection
                              .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                              .getY()),
                      new Rotation2d(poseDirection.getRotation().getRadians()));

              // Pose2d offsetFace = nearestFace.plus(new Transform2d(xOffset, yOffset, new
              // Rotation2d()));
              Logger.recordOutput("reef_face/offset", offsetFace);

              // Transform2d reefOffset = nearestFace.minus(drive.getPose());

              // double theta = Math.atan(reefOffset.getY() / reefOffset.getX());
              // double xGoal = Math.cos(theta) * ALIGN_DISTANCE.baseUnitMagnitude();
              // double yGoal = Math.sin(theta) * ALIGN_DISTANCE.baseUnitMagnitude();

              // Pose2d calculatedReef =
              // nearestFace.transformBy(new Transform2d(xGoal, yGoal, new
              // Rotation2d(theta)));

              // Logger.recordOutput("reef_face/reefOffset", reefOffset);
              // Logger.recordOutput("reef_face/calculatedReef", calculatedReef);

              double yOutput = yController.calculate(drive.getPose().getY(), offsetFace.getY());
              double xOutput = xController.calculate(drive.getPose().getX(), offsetFace.getX());
              double omegaOutput =
                  angleController.calculate(
                      drive.getPose().getRotation().getRadians(),
                      offsetFace.getRotation().getRadians());

              Logger.recordOutput("driveToReef/xError", xController.getPositionError());
              Logger.recordOutput("driveToReef/xPID", xOutput);
              Logger.recordOutput("driveToReef/yError", yController.getPositionError());
              Logger.recordOutput("driveToReef/yPID", yOutput);
              Logger.recordOutput("driveToReef/omegaError", angleController.getPositionError());
              Logger.recordOutput("driveToReef/omegaPID", omegaOutput);

              // double omegaOutput =
              // angleController.calculate(
              // drive.getPose().getRotation().getRadians(),
              // nearestFace.getRotation().getRadians());

              //   double omegaOutput = 0;

              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Apply rotation deadband
              double omegaOverride =
                  MathUtil.applyDeadband(rotationSupplier.getAsDouble(), DEADBAND);

              // Square rotation value for more precise control
              omegaOverride = Math.copySign(omegaOverride * omegaOverride, omegaOverride);

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      (linearVelocity.getX() + xOutput) * drive.getMaxLinearSpeedMetersPerSec(),
                      (linearVelocity.getY() + yOutput) * drive.getMaxLinearSpeedMetersPerSec(),
                      (omegaOutput + omegaOverride) * drive.getMaxAngularSpeedRadPerSec());
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              xController.reset(drive.getPose().getX());
              yController.reset(drive.getPose().getY());
              angleController.reset(drive.getPose().getRotation().getRadians());
            });
  }
}
