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

package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.Mode;
import frc.robot.Constants.TunerConstants25;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.VisionObservation;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Drive extends SubsystemBase {

  public Pose2d pathPlannerOverride = new Pose2d();
  // TunerConstants25 doesn't include these constants, so they are declared locally
  public static final double ODOMETRY_FREQUENCY =
      RobotBase.isSimulation()
          ? 50
          : // if were on the canivore, run at a higher frequency, if were in sim, run at a lower
          // frequency
          new CANBus(TunerConstants25.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      // maximum distance from the center of the robot to one of the modules
      // on a square drive base they should all be the same
      Math.max(
          Math.max(
              Math.hypot(
                  TunerConstants25.FrontLeft.LocationX, TunerConstants25.FrontLeft.LocationY),
              Math.hypot(
                  TunerConstants25.FrontRight.LocationX, TunerConstants25.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants25.BackLeft.LocationX, TunerConstants25.BackLeft.LocationY),
              Math.hypot(
                  TunerConstants25.BackRight.LocationX, TunerConstants25.BackRight.LocationY)));

  // PID and FF tuning
  public double kPValue =
      RobotBase.isSimulation() ? SwerveModuleIOSim.DRIVE_KP : TunerConstants25.driveGains.kP;
  public double kDValue =
      RobotBase.isSimulation() ? SwerveModuleIOSim.DRIVE_KD : TunerConstants25.driveGains.kD;
  public double kVValue =
      RobotBase.isSimulation() ? SwerveModuleIOSim.DRIVE_KV : TunerConstants25.driveGains.kV;
  public double kAValue = RobotBase.isSimulation() ? 0 : TunerConstants25.driveGains.kA;
  public double kSValue = RobotBase.isSimulation() ? 0 : TunerConstants25.driveGains.kS;

  public final LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/kP", kPValue);
  public final LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/kD", kDValue);
  public final LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/kV", kVValue);
  public final LoggedNetworkNumber kA = new LoggedNetworkNumber("/Tuning/kA", kAValue);
  public final LoggedNetworkNumber kS = new LoggedNetworkNumber("/Tuning/kS", kSValue);

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = Units.lbsToKilograms(115);
  private static final double ROBOT_MOI = 4.2; // TODO find these //4.23321575 MOI from cad
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants25.kWheelRadius.in(Meters),
              TunerConstants25.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants25.FrontLeft.DriveMotorGearRatio),
              TunerConstants25.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private ScoreSide desiredScoreSide = ScoreSide.FRONT;
  private double reefAlignAdjustY = -1;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public Drive(
      GyroIO gyroIO,
      SwerveModuleIO flModuleIO,
      SwerveModuleIO frModuleIO,
      SwerveModuleIO blModuleIO,
      SwerveModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new SwerveModule(flModuleIO, 0, TunerConstants25.FrontLeft);
    modules[1] = new SwerveModule(frModuleIO, 1, TunerConstants25.FrontRight);
    modules[2] = new SwerveModule(blModuleIO, 2, TunerConstants25.BackLeft);
    modules[3] = new SwerveModule(brModuleIO, 3, TunerConstants25.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(50.0, 0.0, 0.0), new PIDConstants(7.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);

    PPHolonomicDriveController.overrideXFeedback(
        // Calculate feedback from your custom PID controller
        () -> {
          return pathPlannerOverride.getX();
        });
    // Override the Y feedback
    PPHolonomicDriveController.overrideYFeedback(
        () -> {
          return pathPlannerOverride.getY();
        });

    // Override the rotation feedback
    PPHolonomicDriveController.overrideRotationFeedback(
        () -> {
          // Calculate feedback from your custom PID controller
          return pathPlannerOverride.getRotation().getRadians();
        });

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);

      // update pid values
      if (kP.get() != kPValue
          || kD.get() != kDValue
          || kV.get() != kVValue
          || kA.get() != kAValue
          || kS.get() != kSValue) {
        kPValue = kP.get();
        kDValue = kD.get();
        kVValue = kV.get();
        kAValue = kA.get();
        kSValue = kS.get();
        for (var module : modules) {
          module.setPIDandFF(kPValue, kDValue, kVValue, kAValue, kSValue);
        }
      }
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  public void reverseZeroGyro() {
    setPose(new Pose2d(getPose().getTranslation(), Rotation2d.k180deg));
  }

  public Command reverseZeroGyroCommand() {
    return new InstantCommand(
        () -> {
          reverseZeroGyro();
        });
  }

  public void setDesiredScoringSide(ScoreSide side) {
    desiredScoreSide = side;
  }

  public ScoreSide getDesiredScoringSide() {
    return desiredScoreSide;
  }

  public void setAdjustY(double y) {
    reefAlignAdjustY = y;
  }

  public double getAdjustY() {
    return reefAlignAdjustY;
  }

  public boolean isCoralSideDesired() {
    return desiredScoreSide == ScoreSide.CORAL_INTAKE;
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants25.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  // private double maxSpeed = 0;

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds output = kinematics.toChassisSpeeds(getModuleStates());
    // double speed = Math.hypot(output.vxMetersPerSecond, output.vyMetersPerSecond);
    // maxSpeed = Math.max(maxSpeed, speed);
    // Logger.recordOutput("Swerve/MaxSpeedmps", maxSpeed);
    return output;
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public Rotation3d getRotation3d() {
    return new Rotation3d(
        gyroInputs.pitch.getRadians(),
        gyroInputs.roll.getRadians(),
        poseEstimator.getEstimatedPosition().getRotation().getRadians());
  }

  public Rotation3d getRotationRate() {
    return new Rotation3d(
        gyroInputs.rollVelocityRadPerSec,
        gyroInputs.pitchVelocityRadPerSec,
        gyroInputs.yawVelocityRadPerSec);
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants25.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants25.FrontLeft.LocationX, TunerConstants25.FrontLeft.LocationY),
      new Translation2d(
          TunerConstants25.FrontRight.LocationX, TunerConstants25.FrontRight.LocationY),
      new Translation2d(TunerConstants25.BackLeft.LocationX, TunerConstants25.BackLeft.LocationY),
      new Translation2d(TunerConstants25.BackRight.LocationX, TunerConstants25.BackRight.LocationY)
    };
  }

  public void updatePose(VisionObservation visionObservation) {
    poseEstimator.addVisionMeasurement(
        visionObservation.getPose(),
        visionObservation.getTimestamp(),
        visionObservation.getStdDev());
  }
  // poseEstimator.addVisionMeasurement(visionObservation.getPose(),
  // visionObservation.getTimestamp());

  public Pose2d getPPOverride() {
    return pathPlannerOverride;
  }

  public void setPPOverride(Double xOverride, Double yOverride, Double thetaOverride) {
    pathPlannerOverride = new Pose2d(xOverride, yOverride, new Rotation2d(thetaOverride));
    Logger.recordOutput("setPPOverride", pathPlannerOverride);
  }

  public void clearPPOverride() {
    PPHolonomicDriveController.clearFeedbackOverrides();
  }
}
