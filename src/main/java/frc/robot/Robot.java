// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import au.grapplerobotics.CanBridge;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.LimelightFLConstants;
import frc.robot.Constants.TunerConstants25;
import frc.robot.commands.DriveCommands;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  RobotContainer robotContainer;

  private double endEffectorBiasInches = DriveCommands.END_EFFECTOR_BIAS.in(Inches);
  private final LoggedNetworkNumber endEffectorBiasInchesTuner =
      new LoggedNetworkNumber("/Tuning/endEffectorBiasInches", endEffectorBiasInches);

  public Robot() {
    CanBridge.runTCP();
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Check for valid swerve config
    var modules =
        new SwerveModuleConstants[] {
          TunerConstants25.FrontLeft,
          TunerConstants25.FrontRight,
          TunerConstants25.BackLeft,
          TunerConstants25.BackRight
        };
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);

    //   if(endEffectorBiasInches != endEffectorBiasInchesTuner.get()){
    //   endEffectorBiasInches = endEffectorBiasInchesTuner.get();
    // }
    // Distance END_EFFECTOR_BIAS = Inches.of(endEffectorBiasInches);

    // List<Pose2d> faces = Arrays.asList(FieldConstants.Reef.centerFaces);

    //       List<Pose2d> flippedFaces = new ArrayList<>();

    //       for (int j = 0; j < faces.size(); j++) {
    //         flippedFaces.add(AllianceFlipUtil.apply(faces.get(j)));
    //       }

    //       Pose2d nearestFace = robotContainer.drive.getPose().nearest(flippedFaces);
    //       Logger.recordOutput("driveToReef/reef_face/raw", nearestFace);

    //       int faceIndex = -1;
    //       for (int i = 0; i < flippedFaces.size(); i++) {
    //         if (flippedFaces.get(i) == nearestFace) {
    //           faceIndex = i;
    //           break;
    //         }
    //       }

    //       Pose2d poseDirection =
    //           AllianceFlipUtil.apply(
    //               new Pose2d(
    //                   (FieldConstants.Reef.center),
    //                   (Rotation2d.fromDegrees(180 - (60 * faceIndex)))));

    //       Logger.recordOutput("driveToReef/reef_face/poseDirection", poseDirection);
    //       Logger.recordOutput("driveToReef/reef_face/faceIndex", faceIndex);

    //       double diff =
    //           RotationUtil.wrapRot2d(robotContainer.drive.getPose().getRotation())
    //               .minus(poseDirection.getRotation())
    //               .getDegrees();

    //       double transformY = 0;

    //       Rotation2d closestRotation =
    //           robotContainer.drive.getPose().getRotation().minus(Rotation2d.fromDegrees(diff));

    //       if (Math.abs(diff) >= 90) { // use coral side
    //         robotContainer.drive.setDesiredScoringSide(ScoreSide.FRONT);
    //         closestRotation = closestRotation.plus(Rotation2d.k180deg);
    //         transformY = END_EFFECTOR_BIAS.in(Meters);
    //       } else { // use front
    //         robotContainer.drive.setDesiredScoringSide(ScoreSide.CORAL_INTAKE);
    //         transformY = -END_EFFECTOR_BIAS.in(Meters);
    //       }

    //       double adjustX =
    //           DriveCommands.ALIGN_DISTANCE.baseUnitMagnitude() +
    // FieldConstants.Reef.faceToCenter;
    //       // double adjustY = Units.inchesToMeters(0);

    //       Pose2d offsetFaceCCW =
    //           new Pose2d(
    //               poseDirection
    //                   .transformBy(
    //                       new Transform2d(
    //                           adjustX, FieldConstants.Reef.reefToBranchY + transformY, new
    // Rotation2d()))
    //                   .getTranslation(),
    //               poseDirection.getRotation());

    //               Pose2d offsetFaceCW =
    //               new Pose2d(
    //                   poseDirection
    //                       .transformBy(
    //                           new Transform2d(
    //                               adjustX, -FieldConstants.Reef.reefToBranchY + transformY, new
    // Rotation2d()))
    //                       .getTranslation(),
    //                   poseDirection.getRotation());

    //                   Logger.recordOutput("AutoAlignTune/reef_face/offsetCW", offsetFaceCW);
    //       Logger.recordOutput("AutoAlignTune/reef_face/offsetCCW", offsetFaceCCW);

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // robotContainer.limelight.resetGyroLL4(robotContainer.drive);
    // System.out.println(FieldConstants.Reef.offsetPositions.size());
    // System.out.println(FieldConstants.Reef.centerFaces[0].getX());

    // robotContainer.limelightbl.throttleSet(5);
    // robotContainer.limelightbr.throttleSet(5);
    // robotContainer.limelightfl.throttleSet(5);
    // robotContainer.limelightfr.throttleSet(5);

    robotContainer.limelightbl.setPipeline(LimelightFLConstants.apriltagPipelineIndex);
    robotContainer.limelightbr.setPipeline(LimelightFLConstants.apriltagPipelineIndex);
    robotContainer.limelightfl.setPipeline(LimelightFLConstants.apriltagPipelineIndex);
    robotContainer.limelightfr.setPipeline(LimelightFLConstants.apriltagPipelineIndex);

    // robotContainer.limelightbl.resetGyroLL4();
    // robotContainer.limelightbr.resetGyroLL4();

    robotContainer.updateControllerAlerts();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // robotContainer.limelight.resetGyroLL4();
    PPHolonomicDriveController.clearFeedbackOverrides();
    robotContainer.limelightbl.setPipeline(LimelightFLConstants.apriltagPipelineIndex);
    robotContainer.limelightbr.setPipeline(LimelightFLConstants.apriltagPipelineIndex);
    robotContainer.limelightfl.setPipeline(LimelightFLConstants.apriltagPipelineIndex);
    robotContainer.limelightfr.setPipeline(LimelightFLConstants.apriltagPipelineIndex);
    robotContainer.limelightbl.throttleSet(0);
    robotContainer.limelightbr.throttleSet(0);
    robotContainer.limelightfl.throttleSet(0);
    robotContainer.limelightfr.throttleSet(0);
    // robotContainer.limelightbl.resetGyroLL4();
    // robotContainer.limelightbr.resetGyroLL4();

    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    robotContainer.limelightbl.throttleSet(0);
    robotContainer.limelightbr.throttleSet(0);
    robotContainer.limelightfl.throttleSet(0);
    robotContainer.limelightfr.throttleSet(0);

    robotContainer.limelightbl.setPipeline(LimelightFLConstants.apriltagPipelineIndex);
    robotContainer.limelightbr.setPipeline(LimelightFLConstants.apriltagPipelineIndex);
    robotContainer.limelightfl.setPipeline(LimelightFLConstants.apriltagPipelineIndex);
    robotContainer.limelightfr.setPipeline(LimelightFLConstants.apriltagPipelineIndex);

    // robotContainer.limelight.resetGyroLL4();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
