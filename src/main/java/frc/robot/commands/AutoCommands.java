package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.ScoreSide;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.Superstructure.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Enums.BranchSide;
import frc.robot.util.Enums.ScoringLevel;
import frc.robot.util.RotationUtil;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoCommands {
  private static final double TRANSLATION_TOLERANCE = 0.1;
  private static final double THETA_TOLERANCE = 0.1;
  static final Distance END_EFFECTOR_BIAS = Inches.of(3.3 - 2.5); // towards climber

  public static Command fullAutoReefScoreOverride(
      Drive drive,
      Superstructure superstructure,
      EndEffector endEffector,
      BranchSide branchSide,
      ScoringLevel level) {

    List<Pose2d> faces = Arrays.asList(FieldConstants.Reef.centerFaces);

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            2,
            0.0,
            DriveCommands.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveCommands.ANGLE_MAX_VELOCITY, DriveCommands.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    ProfiledPIDController xController =
        new ProfiledPIDController(
            4, 0.0, DriveCommands.DRIVE_KDX, new TrapezoidProfile.Constraints(3, 3.0));

    ProfiledPIDController yController =
        new ProfiledPIDController(
            4, 0.0, DriveCommands.DRIVE_KDY, new TrapezoidProfile.Constraints(3, 3.0));
    angleController.setTolerance(THETA_TOLERANCE);
    xController.setTolerance(TRANSLATION_TOLERANCE);
    yController.setTolerance(TRANSLATION_TOLERANCE);

    Timer timer = new Timer();

    return Commands.run(
            () -> {
              timer.start();
              switch (branchSide) {
                case CENTER:
                  drive.setAdjustY(0);
                  break;
                case CLOCKWISE:
                  drive.setAdjustY(-FieldConstants.Reef.reefToBranchY);
                  break;
                case COUNTER_CLOCKWISE:
                  drive.setAdjustY(FieldConstants.Reef.reefToBranchY);
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
                  DriveCommands.ALIGN_DISTANCE.baseUnitMagnitude()
                      + FieldConstants.Reef.faceToCenter;
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
              // ChassisSpeeds speeds = new ChassisSpeeds(xOutput, yOutput, omegaOutput);
              // speeds = ChassisSpeeds.discretize(speeds, Constants.loopPeriodSecs);
              // boolean isFlipped = false;

              // DriverStation.getAlliance().isPresent()
              // && DriverStation.getAlliance().get() == Alliance.Red;
              // drive.runVelocity(
              // ChassisSpeeds.fromFieldRelativeSpeeds(
              // speeds,
              // isFlipped
              // ? drive.getRotation().plus(new Rotation2d(Math.PI))
              // : drive.getRotation()));

              // Convert to field relative speeds & send command

              drive.setPPOverride(xOutput, yOutput, omegaOutput);
              Logger.recordOutput("ppoverride/output/x", xOutput);
              Logger.recordOutput("ppoverride/output/y", yOutput);
              Logger.recordOutput("ppoverride/output/theta", omegaOutput);
              Logger.recordOutput("ppoverride/swerve", drive.getPPOverride());

              if ((xController.getPositionError() < 1)
                  && (yController.getPositionError() < 1)
                  && (angleController.getPositionError() < Math.toRadians(90))) {
                SuperstructureActions.prepScore(
                    level, drive::isCoralSideDesired, superstructure, endEffector);
              }
              Logger.recordOutput("Auto/AlignXError", xController.getPositionError());
              Logger.recordOutput("Auto/AlignYError", yController.getPositionError());
              Logger.recordOutput("Auto/AlignThetaError", angleController.getPositionError());
              Logger.recordOutput(
                  "Auto/Aligned",
                  xController.atSetpoint()
                      && (yController.atSetpoint())
                      && (angleController.atSetpoint())
                      && timer.hasElapsed(1));
            })
        // .until(
        // () ->
        // (xController.atSetpoint())
        // && (yController.atSetpoint())
        // && (angleController.atSetpoint())
        // && timer.hasElapsed(1))

        // .andThen(autoScoreNoRetract(superstructure, endEffector,
        // drive::isCoralSideDesired,
        // level))
        .beforeStarting(
            () -> {
              xController.reset(drive.getPose().getX());
              yController.reset(drive.getPose().getY());
              angleController.reset(drive.getPose().getRotation().getRadians());
              drive.setAdjustY(0);
              // drive.setPPOverride(.0, .0, .0);
            })
        .finallyDo(
            // if were not autoaligning, always use the front side, reset adjustY
            () -> {
              drive.setDesiredScoringSide(ScoreSide.FRONT);
              drive.setAdjustY(-1);
              timer.stop();
              timer.reset();
              drive.clearPPOverride();
              Logger.recordOutput("test", true);
            });
  }

  public static Command fullAutoReefScore(
      Drive drive,
      Superstructure superstructure,
      EndEffector endEffector,
      BranchSide branchSide,
      ScoringLevel level,
      boolean isL4) {

    List<Pose2d> faces = Arrays.asList(FieldConstants.Reef.centerFaces);

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            2,
            0.0,
            DriveCommands.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveCommands.ANGLE_MAX_VELOCITY, DriveCommands.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    ProfiledPIDController xController =
        new ProfiledPIDController(
            4, 0.0, DriveCommands.DRIVE_KDX, new TrapezoidProfile.Constraints(3, 3.0));

    ProfiledPIDController yController =
        new ProfiledPIDController(
            4, 0.0, DriveCommands.DRIVE_KDY, new TrapezoidProfile.Constraints(3, 3.0));
    angleController.setTolerance(THETA_TOLERANCE);
    xController.setTolerance(TRANSLATION_TOLERANCE);
    yController.setTolerance(TRANSLATION_TOLERANCE);

    Timer timer = new Timer();

    return Commands.run(
            () -> {
              timer.start();
              switch (branchSide) {
                case CENTER:
                  drive.setAdjustY(0);
                  break;
                case CLOCKWISE:
                  drive.setAdjustY(-FieldConstants.Reef.reefToBranchY);
                  break;
                case COUNTER_CLOCKWISE:
                  drive.setAdjustY(FieldConstants.Reef.reefToBranchY);
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
                  DriveCommands.ALIGN_DISTANCE.baseUnitMagnitude()
                      + FieldConstants.Reef.faceToCenter;
              // double adjustY = Units.inchesToMeters(0);

              Pose2d offsetFace =
                  new Pose2d(
                      poseDirection
                          .transformBy(
                              new Transform2d(
                                  adjustX, drive.getAdjustY() + transformY, new Rotation2d()))
                          .getTranslation(),
                      poseDirection.getRotation());
              Logger.recordOutput("driveToReef/adjustY", drive.getAdjustY());

              Logger.recordOutput("driveToReef/reef_face/adjustY", drive.getAdjustY());
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

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds = new ChassisSpeeds(xOutput, yOutput, omegaOutput);
              speeds = ChassisSpeeds.discretize(speeds, Constants.loopPeriodSecs);
              boolean isFlipped = false;

              // DriverStation.getAlliance().isPresent()
              // && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));

              // Convert to field relative speeds & send command
              // drive.sePPOverride(xOutput, yOutput, omegaOutput);
              // Logger.recordOutput("ppoverride/x", xOutput);
              // Logger.recordOutput("ppoverride/y", yOutput);
              // Logger.recordOutput("ppoverride/theta", omegaOutput);

              // if ((xController.getPositionError() < 0.4)
              // && (yController.getPositionError() < 0.4)
              // && (angleController.getPositionError() < Math.toRadians(2))) {
              // SuperstructureActions.prepScore(
              // level, drive::isCoralSideDesired, superstructure, endEffector);
              // }
              Logger.recordOutput("Auto/AlignXError", xController.getPositionError());
              Logger.recordOutput("Auto/AlignYError", yController.getPositionError());
              Logger.recordOutput("Auto/AlignThetaError", angleController.getPositionError());
              Logger.recordOutput(
                  "Auto/Aligned",
                  xController.atSetpoint()
                      && (yController.atSetpoint())
                      && (angleController.atSetpoint())
                      && timer.hasElapsed(1));
            },
            drive)
        .until(
            () ->
                (xController.atSetpoint())
                    && (yController.atSetpoint())
                    && (angleController.atSetpoint())
                    && timer.hasElapsed(1))
        .andThen(drive3Reef(drive, level, superstructure, endEffector, isL4))
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
              timer.stop();
              timer.reset();
              // drive.clearPPOverride();
              // Logger.recordOutput("test", true);
            });
  }

  /** go to the desired level's corresponding prep position, */
  private static Command autoScoreNoRetract(
      Superstructure superstructure,
      EndEffector endEffector,
      BooleanSupplier flipped,
      ScoringLevel level) {

    return superstructure
        .setState(SuperstructureState.RIGHT_SIDE_UP_IDLE, flipped)
        .andThen(
            endEffector
                .coralOut(() -> level)
                .raceWith(superstructure.setState(level.postState, flipped)));
  }

  /**
   * Picks up a peice that is indexed in the carwash
   *
   * <p>This command assumes that the peice is already there, it will NOT check to see if a peice is
   * indexed
   *
   * <p>After it picks up the peice, the arm will flip around to the coral idle spot
   */
  public static Command intakeCoralGround(
      Superstructure superstructure, CoralIntake intake, Supplier<Angle> trim) {
    return superstructure
        .setState(SuperstructureState.UPSIDE_DOWN_IDLE, () -> false)
        .alongWith(
            new RunCommand(
                () -> {
                  intake.deploy(trim.get());
                  intake.runIn();
                },
                intake))
        .until(intake::hasPiece)
        .andThen(
            new ParallelDeadlineGroup(
                new WaitCommand(1),
                new RunCommand(
                    () -> {
                      intake.retract();
                    },
                    intake)))
        .finallyDo(
            () -> {
              intake.retract();
              intake.dampenCoral();
            })
        .handleInterrupt(
            () -> {
              intake.dampenCoral();
              intake.retract();
            });
  }

  public static Command drive2Reef(
      Drive drive,
      BranchSide side,
      ScoringLevel level,
      Superstructure superstructure,
      EndEffector endEffector) {

    List<Pose2d> faces = Arrays.asList(FieldConstants.Reef.centerFaces);

    ProfiledPIDController angleController =
        new ProfiledPIDController(2, 0.0, 0, new TrapezoidProfile.Constraints(8, 4));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    ProfiledPIDController xController =
        new ProfiledPIDController(3, 0.0, 0, new TrapezoidProfile.Constraints(5, 3.0));

    ProfiledPIDController yController =
        new ProfiledPIDController(5, 0.0, 0, new TrapezoidProfile.Constraints(5, 3.0));

    return Commands.run(
            () -> {
              switch (side) {
                case CENTER:
                  drive.setAdjustY(0);
                  break;
                case CLOCKWISE:
                  drive.setAdjustY(-FieldConstants.Reef.reefToBranchY);
                  break;
                case COUNTER_CLOCKWISE:
                  drive.setAdjustY(FieldConstants.Reef.reefToBranchY);
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
                  DriveCommands.ALIGN_DISTANCE.baseUnitMagnitude()
                      + FieldConstants.Reef.faceToCenter;

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
              ChassisSpeeds speeds = new ChassisSpeeds(xOutput, yOutput, omegaOutput);

              boolean isFlipped = false;
              // DriverStation.getAlliance().isPresent()
              // && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));

              if (Math.abs(xController.getPositionError()) < 0.4
                  && Math.abs(yController.getPositionError()) < 0.4
                  && Math.abs(angleController.getPositionError()) < Math.toRadians(2)) {
                SuperstructureActions.prepScore(
                    level, drive::isCoralSideDesired, superstructure, endEffector);
              }
            },
            drive)
        .andThen(
            superstructure.setState(
                SuperstructureState.POST_CORAL_SCORE_L4,
                drive::isCoralSideDesired,
                endEffector::hasPiece))
        .andThen(
            superstructure.setState(
                SuperstructureState.RIGHT_SIDE_UP_IDLE,
                drive::isCoralSideDesired,
                endEffector::hasPiece))
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

  public static Command drive3Reef(
      Drive drive,
      ScoringLevel level,
      Superstructure superstructure,
      EndEffector endEffector,
      boolean isL4) {

    return SuperstructureActions.prepScore(
            level, drive::isCoralSideDesired, superstructure, endEffector)
        .andThen(superstructure.score(drive::isCoralSideDesired, endEffector::hasPiece))
        .andThen(
            new ConditionalCommand(
                superstructure
                    .setState(SuperstructureState.RIGHT_SIDE_UP_IDLE, endEffector::hasPiece)
                    .alongWith(endEffector.scoreCommand(superstructure::getState))
                    .andThen(endEffector.idleCoralCommand().unless(superstructure::isInPrepState))
                    .alongWith(new InstantCommand(() -> Logger.recordOutput("hmmm", isL4))),
                superstructure
                    .setState(SuperstructureState.POST_CORAL_SCORE_L4, endEffector::hasPiece)
                    .withTimeout(3)
                    .andThen(
                        superstructure.setState(
                            SuperstructureState.RIGHT_SIDE_UP_IDLE, endEffector::hasPiece))
                    .alongWith(endEffector.scoreCommand(superstructure::getState))
                    .andThen(endEffector.idleCoralCommand().unless(superstructure::isInPrepState))
                    .alongWith(new InstantCommand(() -> Logger.recordOutput("hmmm", isL4))),
                () -> !isL4));
  }
}
