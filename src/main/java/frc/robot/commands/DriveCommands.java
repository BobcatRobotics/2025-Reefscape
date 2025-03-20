package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.ScoreSide;
import frc.robot.subsystems.PhotonVision.Photon;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.RotationUtil;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  static final double ANGLE_KP = .5;
  static final double ANGLE_KD = 0;
  static final double DRIVE_KPY = 1;
  static final double DRIVE_KDY = 0;
  static final double DRIVE_KPX = 1;
  static final double DRIVE_KDX = 0;
  static final double ANGLE_MAX_VELOCITY = 8.0;
  static final double ANGLE_MAX_ACCELERATION = 4;
  static final Distance ALIGN_DISTANCE = Inches.of(16); // TODO this should be zero

  static final Distance END_EFFECTOR_BIAS = Inches.of(3.3 - 1); // towards climber

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
      DoubleSupplier omegaSupplier,
      DoubleSupplier elevatorHeightPercentage,
      DoubleSupplier fineStrafeXSupplier,
      DoubleSupplier fineStrafeYSupplier) {
    return Commands.run(
        () -> {
          double maxSpeed = drive.getMaxLinearSpeedMetersPerSec();
          boolean useFineStrafe =
              Math.abs(fineStrafeXSupplier.getAsDouble()) > 0.25
                  || Math.abs(fineStrafeYSupplier.getAsDouble()) > 0.25; // TODO verify deadzone

          if (useFineStrafe) {
            maxSpeed = maxSpeed * 0.25;
          }
          // TODO finalize this step
          if (DriverStation.isTeleop()) {
            if (elevatorHeightPercentage.getAsDouble() > 0.75) {
              maxSpeed = maxSpeed * 0.5;
            } else if (elevatorHeightPercentage.getAsDouble() > 0.5) {
              maxSpeed = maxSpeed * 0.75;
            }
          }

          // Get linear velocity
          Translation2d linearVelocity =
              useFineStrafe
                  ? getLinearVelocityFromJoysticks(
                      fineStrafeXSupplier.getAsDouble(), fineStrafeYSupplier.getAsDouble())
                  : getLinearVelocityFromJoysticks(
                      xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * maxSpeed,
                  linearVelocity.getY() * maxSpeed,
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped = // true;
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Blue;
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
      BooleanSupplier ccwSupplier,
      BooleanSupplier cwSupplier,
      DoubleSupplier aidenAlignX,
      DoubleSupplier aidenAlignY) {

    List<Pose2d> faces = Arrays.asList(FieldConstants.Reef.centerFaces);

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            2,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    ProfiledPIDController xController =
        new ProfiledPIDController(3, 0.0, DRIVE_KDX, new TrapezoidProfile.Constraints(5, 3.0));

    ProfiledPIDController yController =
        new ProfiledPIDController(5, 0.0, DRIVE_KDY, new TrapezoidProfile.Constraints(5, 3.0));

    return Commands.run(
            () -> {
              if (ccwSupplier.getAsBoolean()) {
                drive.setAdjustY(FieldConstants.Reef.reefToBranchY);
              } else if (cwSupplier.getAsBoolean()) {
                drive.setAdjustY(-FieldConstants.Reef.reefToBranchY);
              }

              List<Pose2d> flippedFaces = new ArrayList<>();

              for (int j = 0; j < faces.size(); j++) {
                flippedFaces.add(AllianceFlipUtil.apply(faces.get(j)));
              }

              Pose2d nearestFace = drive.getPose().nearest(flippedFaces);
              Logger.recordOutput("driveToReef/reef_face/raw", nearestFace);

              int faceIndex = -1;
              for (int i = 0; i < flippedFaces.size(); i++) {
                if (flippedFaces.get(i) == nearestFace) {
                  faceIndex = i;
                  break;
                }
              }

              Pose2d poseDirection =
                  AllianceFlipUtil.apply(
                      new Pose2d(
                          (FieldConstants.Reef.center),
                          (Rotation2d.fromDegrees(180 - (60 * faceIndex)))));

              Logger.recordOutput("driveToReef/reef_face/poseDirection", poseDirection);
              Logger.recordOutput("driveToReef/reef_face/faceIndex", faceIndex);

              double diff =
                  RotationUtil.wrapRot2d(drive.getPose().getRotation())
                      .minus(poseDirection.getRotation())
                      .getDegrees();

              double transformY = 0;

              Rotation2d closestRotation =
                  drive.getPose().getRotation().minus(Rotation2d.fromDegrees(diff));

              if (Math.abs(diff) >= 90) { // use coral side
                drive.setDesiredScoringSide(ScoreSide.FRONT);
                closestRotation = closestRotation.plus(Rotation2d.k180deg);
                transformY = END_EFFECTOR_BIAS.in(Meters);
              } else { // use front
                drive.setDesiredScoringSide(ScoreSide.CORAL_INTAKE);
                transformY = -END_EFFECTOR_BIAS.in(Meters);
              }

              double adjustX =
                  ALIGN_DISTANCE.baseUnitMagnitude() + FieldConstants.Reef.faceToCenter;
              // double adjustY = Units.inchesToMeters(0);

              Pose2d offsetFace =
                  new Pose2d(
                      poseDirection
                          .transformBy(
                              new Transform2d(
                                  adjustX, drive.getAdjustY() + transformY, new Rotation2d()))
                          .getTranslation(),
                      poseDirection.getRotation());
              Logger.recordOutput("adjustY", drive.getAdjustY());

              Logger.recordOutput("driveToReef/reef_face/adjustY", drive.getAdjustY());
              Logger.recordOutput("driveToReef/reef_face/transformY", transformY);

              Logger.recordOutput("driveToReef/reef_face/offset", offsetFace);

              double yOutput = yController.calculate(drive.getPose().getY(), offsetFace.getY());
              double xOutput = xController.calculate(drive.getPose().getX(), offsetFace.getX());
              double omegaOutput =
                  angleController.calculate(
                      drive.getPose().getRotation().getRadians(), closestRotation.getRadians());

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

              // double omegaOutput = 0;

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
                      (xOutput * (1 - Math.abs(linearVelocity.getX()))),
                      // + (linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec()),
                      (yOutput * (1 - Math.abs(linearVelocity.getY()))),
                      // + (linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec()),
                      (omegaOutput * (1 - Math.abs(omegaOverride))));
              // + (omegaOverride * drive.getMaxLinearSpeedMetersPerSec()));

              ChassisSpeeds overrideSpeeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omegaOverride * drive.getMaxAngularSpeedRadPerSec());
              boolean isFlipped = // false;
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Blue;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                          speeds,
                          // isFlipped
                          // ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          // :
                          drive.getRotation())
                      // AidenAlign
                      .plus(
                          new ChassisSpeeds(
                              aidenAlignX.getAsDouble(), aidenAlignY.getAsDouble(), 0))
                      .plus(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              overrideSpeeds,
                              isFlipped
                                  ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                  : drive.getRotation())));
            },
            drive)
        .beforeStarting(
            () -> {
              xController.reset(drive.getPose().getX());
              yController.reset(drive.getPose().getY());
              angleController.reset(drive.getPose().getRotation().getRadians());
              drive.setAdjustY(0);
            })
        .finallyDo(
            // if were not autoaligning, always use the front side, reset adjustY
            () -> {
              drive.setDesiredScoringSide(ScoreSide.FRONT);
              drive.setAdjustY(-1);
            });
  }

  public static Command driveToCoral(
      Drive drive,
      Photon photon,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier angleSupplier,
      DoubleSupplier elevatorHeightPercentage) {

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            .1,
            0.0,
            0,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-180, 180);
    LinearFilter omegaFilter = LinearFilter.movingAverage(5);

    // ProfiledPIDController distanceController =
    // new ProfiledPIDController(
    // DRIVE_KPX, 0.0, DRIVE_KDX, new TrapezoidProfile.Constraints(1, 1.0));
    // LinearFilter xFilter = LinearFilter.movingAverage(50);
    return Commands.run(
            () -> {
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              double StickMagnitude = linearVelocity.getDistance(new Translation2d(0, 0));
              List<Double> areaList = new ArrayList<Double>();
              PhotonPipelineResult result = photon.result();

              for (PhotonTrackedTarget target : result.targets) {
                if (target.objDetectId == 1) {
                  areaList.add(target.area);
                } else {
                  areaList.add(0.0);
                }
              }

              int maxIndex = 0;
              double maxValue = 0;

              for (int i = 1; i < areaList.size(); i++) {
                if (areaList.get(i) > maxValue) {
                  maxValue = areaList.get(i);
                  maxIndex = i;
                }
              }
              double omega = 0;
              if (areaList.size() != 0) {
                PhotonTrackedTarget target = result.targets.get(maxIndex);

                omega = target.yaw;
              }
              Logger.recordOutput("CoralDetect/yaw", omega);
              // double distToTag = distanceSupplier.getAsDouble();
              // double omega = omegaSupplier.get().getRadians();
              // double filteredDistance = 0;
              double filteredOmega = 0;

              // if (distToTag != 0) {
              // filteredDistance = xFilter.calculate(distToTag);
              // Logger.recordOutput("SingleTagAlign/filteredDistance", filteredDistance);
              // }
              // if (omega != 0) {
              // filteredOmega = omegaFilter.calculate(omega);
              // }
              filteredOmega = omega;
              Logger.recordOutput("SingleTagAlign/filteredOmega", filteredOmega);

              // double omegaOutput =
              // filteredOmega == 0 ? 0 : angleController.calculate(filteredOmega, 0);

              double omegaOutput = angleController.calculate(filteredOmega, 0);

              Logger.recordOutput("CoralDetect/omegaOutput", omegaOutput);
              Logger.recordOutput("CoralDetect/omegaError", angleController.getPositionError());

              double xOutput = 0;
              if (filteredOmega != 0) {
                xOutput = .5 + (1 / Math.abs(filteredOmega));
              } else if (filteredOmega == 0 && maxValue != 0) {
                xOutput = .75;
              }

              // double distanceOutput = distToTag == 0 ? 0 :
              // distanceController.calculate(filteredDistance, 2);

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
                      -xOutput, // -(StickMagnitude * drive.getMaxLinearSpeedMetersPerSec()) +
                      // xOutput,
                      0,
                      omegaOutput); // * drive.getMaxAngularSpeedRadPerSec());
              drive.runVelocity(speeds);
            },
            drive)
        .unless(() -> !photon.hasCoral())
        .beforeStarting(
            () -> {
              angleController.reset(0);
            });
  }

  public static Command driveToProcessor(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      DoubleSupplier aidenAlignX,
      DoubleSupplier aidenAlignY) {

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            2,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    ProfiledPIDController xController =
        new ProfiledPIDController(3, 0.0, DRIVE_KDX, new TrapezoidProfile.Constraints(5, 3.0));

    ProfiledPIDController yController =
        new ProfiledPIDController(2, 0.0, DRIVE_KDY, new TrapezoidProfile.Constraints(5, 3.0));

    return Commands.run(
            () -> {
              Pose2d nearestFace = AllianceFlipUtil.apply(FieldConstants.Processor.centerFace);
              Logger.recordOutput("processor_face/raw", nearestFace);

              // List<Map<ReefHeight, Pose3d>> offsetPositions =
              // FieldConstants.Reef.branchPositions;

              // double xOffset =
              // ALIGN_DISTANCE.baseUnitMagnitude()
              // * Math.cos(nearestFace.getRotation().getRadians());
              // double yOffset =
              // ALIGN_DISTANCE.baseUnitMagnitude()
              // * Math.sin(nearestFace.getRotation().getRadians());

              // Pose2d offsetFace =
              // nearestFace.plus(new Transform2d(xOffset, yOffset, new Rotation2d()));
              // Logger.recordOutput("reef_face/offset", offsetFace);

              double adjustX = ALIGN_DISTANCE.in(Meters);

              double transformY = -END_EFFECTOR_BIAS.in(Meters);
              // double adjustY = Units.inchesToMeters(0);

              Pose2d offsetFace =
                  new Pose2d(
                      nearestFace
                          .transformBy(new Transform2d(adjustX, transformY, new Rotation2d()))
                          .getTranslation(),
                      nearestFace.getRotation());

              Logger.recordOutput("processor_face/adjustY", drive.getAdjustY());

              Logger.recordOutput("processor_face/adjustY", drive.getAdjustY());
              Logger.recordOutput("reef_face/offset", offsetFace);

              double yOutput = yController.calculate(drive.getPose().getY(), offsetFace.getY());
              double xOutput = xController.calculate(drive.getPose().getX(), offsetFace.getX());
              double omegaOutput =
                  angleController.calculate(
                      drive.getPose().getRotation().getRadians(),
                      offsetFace.getRotation().getRadians() + Math.PI);

              Logger.recordOutput("processor_face/xError", xController.getPositionError());
              Logger.recordOutput("processor_face/xPID", xOutput);
              Logger.recordOutput("processor_face/yError", yController.getPositionError());
              Logger.recordOutput("processor_face/yPID", yOutput);
              Logger.recordOutput("processor_face/omegaError", angleController.getPositionError());
              Logger.recordOutput("processor_face/omegaPID", omegaOutput);

              // double omegaOutput =
              // angleController.calculate(
              // drive.getPose().getRotation().getRadians(),
              // nearestFace.getRotation().getRadians());

              // double omegaOutput = 0;

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
                      (xOutput * (1 - linearVelocity.getX()))
                          + (linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec()),
                      (yOutput * (1 - linearVelocity.getY()))
                          + (linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec()),
                      (omegaOutput * (1 - omegaOverride))
                          + (omegaOverride * drive.getMaxLinearSpeedMetersPerSec()));
              boolean isFlipped = false;
              // DriverStation.getAlliance().isPresent()
              //     && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                          speeds,
                          isFlipped
                              ? drive.getRotation().plus(new Rotation2d(Math.PI))
                              : drive.getRotation())
                      // AidenAlign
                      .plus(
                          new ChassisSpeeds(
                              aidenAlignX.getAsDouble(), aidenAlignY.getAsDouble(), 0)));
            },
            drive)
        .beforeStarting(
            () -> {
              xController.reset(drive.getPose().getX());
              yController.reset(drive.getPose().getY());
              angleController.reset(drive.getPose().getRotation().getRadians());
            })
        .finallyDo(
            // if were not autoaligning, always use the front side, reset adjustY
            () -> {
              // drive.setDesiredScoringSide(ScoreSide.FRONT);
              // drive.setAdjustY(0);
            });
  }

  public static Command driveToBarge(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      DoubleSupplier aidenAlignX,
      DoubleSupplier aidenAlignY) {

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            2,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // ProfiledPIDController xController =
    //     new ProfiledPIDController(3, 0.0, DRIVE_KDX, new TrapezoidProfile.Constraints(5, 3.0));

    // ProfiledPIDController yController =
    //     new ProfiledPIDController(2, 0.0, DRIVE_KDY, new TrapezoidProfile.Constraints(5, 3.0));

    return Commands.run(
            () -> {
              // Pose2d nearestFace = AllianceFlipUtil.apply(FieldConstants.Processor.centerFace);
              // Logger.recordOutput("processor_face/raw", nearestFace);

              // List<Map<ReefHeight, Pose3d>> offsetPositions =
              // FieldConstants.Reef.branchPositions;

              // double xOffset =
              // ALIGN_DISTANCE.baseUnitMagnitude()
              // * Math.cos(nearestFace.getRotation().getRadians());
              // double yOffset =
              // ALIGN_DISTANCE.baseUnitMagnitude()
              // * Math.sin(nearestFace.getRotation().getRadians());

              // Pose2d offsetFace =
              // nearestFace.plus(new Transform2d(xOffset, yOffset, new Rotation2d()));
              // Logger.recordOutput("reef_face/offset", offsetFace);

              // double adjustX = ALIGN_DISTANCE.in(Meters);

              // double transformY = -END_EFFECTOR_BIAS.in(Meters);
              // double adjustY = Units.inchesToMeters(0);

              // Pose2d offsetFace =
              //     new Pose2d(
              //         nearestFace
              //             .transformBy(new Transform2d(adjustX, transformY, new Rotation2d()))
              //             .getTranslation(),
              //         nearestFace.getRotation());

              // Logger.recordOutput("processor_face/adjustY", drive.getAdjustY());

              // Logger.recordOutput("processor_face/adjustY", drive.getAdjustY());
              // Logger.recordOutput("reef_face/offset", offsetFace);

              // double yOutput = yController.calculate(drive.getPose().getY(), offsetFace.getY());
              // double xOutput = xController.calculate(drive.getPose().getX(), offsetFace.getX());

              Rotation2d omegaGoal = new Rotation2d(AllianceFlipUtil.shouldFlip() ? Math.PI : 0);

              double diff =
                  RotationUtil.wrapRot2d(drive.getPose().getRotation())
                      .minus(omegaGoal)
                      .getDegrees();

              Rotation2d closestRotation =
                  drive.getPose().getRotation().minus(Rotation2d.fromDegrees(diff));

              if (Math.abs(diff) >= 90) { // use coral side
                drive.setDesiredScoringSide(ScoreSide.FRONT);
                closestRotation = closestRotation.plus(Rotation2d.k180deg);
              } else { // use front
                drive.setDesiredScoringSide(ScoreSide.CORAL_INTAKE);
              }

              double omegaOutput =
                  angleController.calculate(
                      drive.getPose().getRotation().getRadians(), closestRotation.getRadians());

              // Logger.recordOutput("processor_face/xError", xController.getPositionError());
              // Logger.recordOutput("processor_face/xPID", xOutput);
              // Logger.recordOutput("processor_face/yError", yController.getPositionError());
              // Logger.recordOutput("processor_face/yPID", yOutput);
              Logger.recordOutput("barge/omegaError", angleController.getPositionError());
              Logger.recordOutput("barge/omegaPID", omegaOutput);

              // double omegaOutput =
              // angleController.calculate(
              // drive.getPose().getRotation().getRadians(),
              // nearestFace.getRotation().getRadians());

              // double omegaOutput = 0;

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
                      (linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec()),
                      (linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec()),
                      (omegaOutput * (1 - omegaOverride))
                          + (omegaOverride * drive.getMaxLinearSpeedMetersPerSec()));
              boolean isFlipped = false;
              // DriverStation.getAlliance().isPresent()
              //     && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                          speeds,
                          isFlipped
                              ? drive.getRotation().plus(new Rotation2d(Math.PI))
                              : drive.getRotation())
                      // AidenAlign
                      .plus(
                          new ChassisSpeeds(
                              aidenAlignX.getAsDouble(), aidenAlignY.getAsDouble(), 0)));
            },
            drive)
        .beforeStarting(
            () -> {
              // xController.reset(drive.getPose().getX());
              // yController.reset(drive.getPose().getY());
              angleController.reset(drive.getPose().getRotation().getRadians());
            })
        .finallyDo(
            // if were not autoaligning, always use the front side, reset adjustY
            () -> {
              drive.setDesiredScoringSide(ScoreSide.FRONT);
              // drive.setAdjustY(0);
            });
  }

  public static Command driveToReefAuto(Drive drive, Boolean Counterclockwise) {

    List<Pose2d> faces = Arrays.asList(FieldConstants.Reef.centerFaces);

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            2,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    ProfiledPIDController xController =
        new ProfiledPIDController(3, 0.0, DRIVE_KDX, new TrapezoidProfile.Constraints(5, 3.0));

    ProfiledPIDController yController =
        new ProfiledPIDController(2, 0.0, DRIVE_KDY, new TrapezoidProfile.Constraints(5, 3.0));

    return Commands.run(
            () -> {
              if (Counterclockwise) {
                drive.setAdjustY(FieldConstants.Reef.reefToBranchY);
              } else {
                drive.setAdjustY(-FieldConstants.Reef.reefToBranchY);
              }

              List<Pose2d> flippedFaces = new ArrayList<>();

              for (int j = 0; j < faces.size(); j++) {
                flippedFaces.add(AllianceFlipUtil.apply(faces.get(j)));
              }

              Pose2d nearestFace = drive.getPose().nearest(flippedFaces);
              Logger.recordOutput("reef_face/raw", nearestFace);

              int faceIndex = -1;
              for (int i = 0; i < flippedFaces.size(); i++) {
                if (flippedFaces.get(i) == nearestFace) {
                  faceIndex = i;
                  break;
                }
              }

              Pose2d poseDirection =
                  AllianceFlipUtil.apply(
                      new Pose2d(
                          (FieldConstants.Reef.center),
                          (Rotation2d.fromDegrees(180 - (60 * faceIndex)))));

              Logger.recordOutput("reef_face/poseDirection", poseDirection);
              Logger.recordOutput("reef_face/faceIndex", faceIndex);

              double diff =
                  RotationUtil.wrapRot2d(drive.getPose().getRotation())
                      .minus(poseDirection.getRotation())
                      .getDegrees();

              double transformY = 0;

              Rotation2d closestRotation =
                  drive.getPose().getRotation().minus(Rotation2d.fromDegrees(diff));

              if (Math.abs(diff) >= 90) { // use coral side
                drive.setDesiredScoringSide(ScoreSide.FRONT);
                closestRotation = closestRotation.plus(Rotation2d.k180deg);
                transformY = END_EFFECTOR_BIAS.in(Meters);
              } else { // use front
                drive.setDesiredScoringSide(ScoreSide.CORAL_INTAKE);
                transformY = -END_EFFECTOR_BIAS.in(Meters);
              }

              double adjustX =
                  ALIGN_DISTANCE.baseUnitMagnitude() + FieldConstants.Reef.faceToCenter;
              // double adjustY = Units.inchesToMeters(0);

              Pose2d offsetFace =
                  new Pose2d(
                      poseDirection
                          .transformBy(
                              new Transform2d(
                                  adjustX, drive.getAdjustY() + transformY, new Rotation2d()))
                          .getTranslation(),
                      poseDirection.getRotation());
              Logger.recordOutput("adjustY", drive.getAdjustY());

              Logger.recordOutput("reef_face/adjustY", drive.getAdjustY());
              Logger.recordOutput("reef_face/offset", offsetFace);

              double yOutput = yController.calculate(drive.getPose().getY(), offsetFace.getY());
              double xOutput = xController.calculate(drive.getPose().getX(), offsetFace.getX());
              double omegaOutput =
                  angleController.calculate(
                      drive.getPose().getRotation().getRadians(), closestRotation.getRadians());

              Logger.recordOutput("driveToReef/xError", xController.getPositionError());
              Logger.recordOutput("driveToReef/xPID", xOutput);
              Logger.recordOutput("driveToReef/yError", yController.getPositionError());
              Logger.recordOutput("driveToReef/yPID", yOutput);
              Logger.recordOutput("driveToReef/omegaError", angleController.getPositionError());
              Logger.recordOutput("driveToReef/omegaPID", omegaOutput);

              // Convert to field relative speeds & send command
              // drive.sePPOverride(() -> xOutput, () -> yOutput, () -> omegaOutput);
              Logger.recordOutput("ppoverride/x", xOutput);
              Logger.recordOutput("ppoverride/y", yOutput);
              Logger.recordOutput("ppoverride/theta", omegaOutput);
            })
        .beforeStarting(
            () -> {
              xController.reset(drive.getPose().getX());
              yController.reset(drive.getPose().getY());
              angleController.reset(drive.getPose().getRotation().getRadians());
            })
        .finallyDo(
            // if were not autoaligning, always use the front side, reset adjustY
            () -> {
              drive.setDesiredScoringSide(ScoreSide.FRONT);
              drive.setAdjustY(0);
            });
  }

  public static Command overridePP(
      Drive drive, DoubleSupplier xOutput, DoubleSupplier yOutput, DoubleSupplier omegaOutput) {
    return Commands.run(
            () -> {
              drive.setPPOverride(
                  xOutput.getAsDouble(), yOutput.getAsDouble(), omegaOutput.getAsDouble());
              Logger.recordOutput("ppoverride/x", xOutput.getAsDouble());
              Logger.recordOutput("ppoverride/y", yOutput.getAsDouble());
              Logger.recordOutput("ppoverride/theta", omegaOutput.getAsDouble());
            })
        .finallyDo(() -> drive.clearPPOverride());
  }
}

// The cool thing about being the only one who ever touches certian parts of the
// code is you can
// put whatever you want in the comments and no one will ever bother to read it

// certAIn actually

// ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠛⠛⠛⠛⠛⠛⢛⣿⠿⠟⠛⠛⠛⠛⠛⠛⠿⠿⣿⣟⠛⠛⠛⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠿⠿⣛
// ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣇⠀⠀⠀⠀⢠⡿⠁ ⠀⠀ ⠙⢷⡀⠺⠿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠃⣰⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⢸⡇ AMOGUS XD ⢸⡇⠀⠀⠙⣿⣿⣿⣿⣿⣿⣿⠃⢀⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣄⣀⡀⠀⠘⢷⣀⠀⠀⠀ ⠀⠀⠀⢀⣼⠃⠀⠀⠀⠉⠛⠿⢿⣿⣿⡏⠀⣼⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⡿⠟⠉⠀⡀⠀⠀⠉⠛⢷⣄⠈⢙⡷⠀⠀⣠⣤⣤⣤⣤⣤⡴⠾⠋⠁⣠⡶⠶⠶⠶⠶⣤⡀⠀⣿⡇⠀⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⠏⠀⠀⠐⠉⠉⠁⠀⠀⠀⠀⠹⣶⠻⠟⠛⠛⠋⠀⠀⠀⡏⠀⠀⠀⠀⢠⡏⣠⣤⠤⠤⣄⡈⢻⡄⣿⡇⠀⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⠀⠀⠀⢰⡖⠲⣶⣶⢤⡤⠤⣤⣿⡆⠀⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⡾⠀⠻⣷⣶⡶⠾⠃⠈⣿⣿⡇⠀⣿⡿⠿⢿
// ⣿⣿⣿⣿⣿⣿⣿⣆⠀⠀⢀⣉⣛⠛⠉⢠⡙⠲⢿⣿⠃⠀⠀⠀⠀⠀⠀⢰⠇⠀⠀⠀⢰⡇⠀⠀⠀⠀⠀⠀⠀⠀⢹⣿⡇⠀⣿⣧⣤⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⣷⣤⣌⠛⠿⠿⠖⣎⣤⣶⡛⠁⠀⠀⠀⠀⠀⠀⠀⢸⠀⠀⠀⠀⣾⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⡇⠀⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⠟⠋⠉⠙⠛⠻⣿⣿⣿⣿⣿⣿⣧⠀⠀⠀⠀⠀⠀⠀⢸⠀⠀⠀⢠⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⡇⠀⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⡿⠁⠀⠠⢤⡀⠀⢀⡬⠟⣻⣿⣯⠍⠻⣆⠀⠀⠀⠀⠀⠀⢸⠀⠀⠀⢸⡇⠀⣠⠶⠶⠶⢶⡀⠀⠀⢸⣿⡇⠀⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⠃⠀⡀⠀⠀⠉⠓⠋⠀⠀⣳⣾⡴⠂⠀⢹⡆⠀⠀⠀⠀⢀⣸⣰⣛⣛⣺⣀⣀⣸⣆⣀⣀⣸⣇⣀⣀⣸⣿⡇⠀⣿⣿⣿⣿
// ⣿⣿⣿⣿⡏⠀⠀⠉⠓⢦⣄⣀⣠⣴⣿⣷⣼⣵⣻⡄⠀⡇⠀⠀⠀⠀⢸⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿
// ⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠉⢹⣿⣿⣿⣿⣿⣍⣀⣸⣧⣤⣤⣤⣤⣼⣄⣀⣀⣀⡀⠀⢀⣀⣠⣤⣤⣤⣤⣤⣤⣀⣀⣀⠀⠀⠀⠀⠀
// ⠛⠛⢻⡏⠀⠀⠀⠀⠀⠀⠀⠀⣾⠀⠀⠀⠀⠀⠈⠉⠁⠀⠀⠀⠀⠀⠉⠉⠉⠛⠛⠛⠛⠛⠉⠉⠀⠀⠀⠀⠀⠉⠉⠉⠛⠛⠛⠛⠛⠛
// ⣀⣀⣸⠁⠀⠀⢀⣶⣶⣦⠀⢀⣟⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣠⣄⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀

// pretty sus
