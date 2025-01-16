package frc.robot.Subsystems.Swerve;


import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.RotationUtil;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Swerve.Gyro.GyroIO;
import frc.robot.Subsystems.Swerve.PoseEstimation.AdvancedSwervePoseEstimator;
import frc.robot.Subsystems.Swerve.SwerveModule.SwerveModule;
import frc.robot.Subsystems.Swerve.SwerveModule.SwerveModuleIO;
import frc.robot.Swerve.GyroIOInputsAutoLogged;

public class Swerve extends SubsystemBase {

    public static final double ODOMETRY_FREQUENCY = new CANBus(SwerveConstants.CAN_BUS).isNetworkFD() ? 100.0 : 100.0; // TODO decide if we should change this

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SwerveModule[] modules;
    private final AdvancedSwervePoseEstimator poseEstimator;
    private final Field2d field2d = new Field2d();

    private final PIDController rotationPID;
    private final PIDController autoAlignPID;
    private double lastMovingYaw = 0.0;
    private boolean rotating = false;

    static final Lock odometryLock = new ReentrantLock();

    private Rotation2d lastYaw = new Rotation2d();

    public Swerve(GyroIO gyroIO, SwerveModuleIO flIO, SwerveModuleIO frIO, SwerveModuleIO blIO, SwerveModuleIO brIO) {
        this.gyroIO = gyroIO;
        SmartDashboard.putData(field2d);
        modules = new SwerveModule[] {
                new SwerveModule(flIO, 0),
                new SwerveModule(frIO, 1),
                new SwerveModule(blIO, 2),
                new SwerveModule(brIO, 3)
        };

        PhoenixOdometryThread.getInstance().start();

        rotationPID = new PIDController(SwerveConstants.teleopRotationKP, SwerveConstants.teleopRotationKI,
                SwerveConstants.teleopRotationKD);
        rotationPID.enableContinuousInput(0, 2 * Math.PI);
        autoAlignPID = new PIDController(SwerveConstants.autoAlignRotationKP, SwerveConstants.autoAlignRotationKI,
                SwerveConstants.autoAlignRotationKD);
        autoAlignPID.enableContinuousInput(0, 2 * Math.PI);

        // Using last year's default deviations, need to tune

        poseEstimator = new 
        AdvancedSwervePoseEstimator(
            SwerveConstants.swerveKinematics,
            getYaw(),
            getModulePositions(),
            getPose(),
            SwerveConstants.autostateStdDevs,
            VisionConstants.autoStdDevs 
        );

        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getChassisSpeeds,
                this::drive,
                new PPHolonomicDriveController(
                        new PIDConstants(SwerveConstants.translationKP, SwerveConstants.translationKI,
                                SwerveConstants.translationKD),
                        new PIDConstants(SwerveConstants.rotationKP, SwerveConstants.rotationKI,
                                SwerveConstants.rotationKD)),
                new RobotConfig(
                        RobotConstants.mass,
                        RobotConstants.MOI,
                        new ModuleConfig(
                                SwerveConstants.wheelCircumference,
                                SwerveConstants.maxModuleSpeed,
                                ODOMETRY_FREQUENCY,
                                DCMotor.getKrakenX60Foc(1),
                                SwerveConstants.driveSupplyCurrentLimit,
                                1),
                        SwerveConstants.moduleTranslations),
                () -> false,
                this);
    }

    public void setLastMovingYaw(double value) {
        lastMovingYaw = value;
    }

    public void periodic() {
        odometryLock.lock();
        gyroIO.updateInputs(gyroInputs);
        for (SwerveModule module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        Logger.recordOutput("Swerve/YawSetpoint", lastMovingYaw);
        Logger.recordOutput("Swerve/CurrentYaw", getYaw().getRadians());
        Logger.processInputs("Swerve/Gyro", gyroInputs);

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
            }

            if (gyroInputs.connected) { // Use gyro when connected
                Rotation2d yaw = getYaw();
                lastYaw = yaw;
            } else { // If disconnected or sim, use angular velocity
                Rotation2d yaw = lastYaw.plus(
                        Rotation2d.fromRadians(getChassisSpeeds().omegaRadiansPerSecond * Constants.loopPeriodSecs));
                lastYaw = yaw;
            }

            poseEstimator.updateWithTime(sampleTimestamps[i], lastYaw, modulePositions);
        }

        Logger.recordOutput("Swerve/Pose", getPose());
        field2d.setRobotPose(getPose());

        Logger.recordOutput("Swerve/ChassisSpeeds",
                new Translation2d(ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getYaw()).vxMetersPerSecond,
                        ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getYaw()).vyMetersPerSecond));
        if (DriverStation.isDisabled()) {
            for (SwerveModule mod : modules) {
                mod.stop();
            }
        }
    }

    /**
     * Gets the current yaw of the gyro or the estimated yaw if the gyro is
     * disconnected
     * 
     * @return current yaw of the gyro
     */
    public Rotation2d getYaw() {
        if (gyroInputs.connected) { // Use gyro when connected
            return gyroInputs.yawPosition;
        } else { // If disconnected or sim, use angular velocity
            return lastYaw;
        }
    }

    /**
     * Makes the swerve drive move
     * 
     * @param translation    desired x and y speeds of the swerve drive in meters
     *                       per
     *                       second
     * @param rotation       desired rotation speed of the swerve drive in radians
     *                       per second
     * @param fieldRelative  whether the values should be field relative or not
     * @param angleToSpeaker in radians
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative,
            boolean autoAlign, Rotation2d autoAlignAngle) {

        ChassisSpeeds desiredSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getYaw())
                : new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation);

        if (autoAlign) {
            desiredSpeeds.omegaRadiansPerSecond = autoAlignPID.calculate(RotationUtil.wrap(getYaw()).getRadians(),
                    RotationUtil.wrap(autoAlignAngle).getRadians());
            Logger.recordOutput("Swerve/AutoAlignPID/setpoint", RotationUtil.wrap(autoAlignAngle));
            Logger.recordOutput("Swerve/AutoAlignPID/error", autoAlignPID.getError());
            lastMovingYaw = getYaw().getRadians();
        } else {
            if (rotation == 0) {
                if (rotating) {
                    rotating = false;
                    lastMovingYaw = getYaw().getRadians();
                }
                desiredSpeeds.omegaRadiansPerSecond = rotationPID.calculate(RotationUtil.wrap(getYaw()).getRadians(),
                        lastMovingYaw);
            } else {
                rotating = true;
            }
        }

        desiredSpeeds = ChassisSpeeds.discretize(desiredSpeeds, Constants.loopPeriodSecs);
        Logger.recordOutput("Swerve/desiredChassisSpeeds", desiredSpeeds);
        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(desiredSpeeds);
        // SwerveModuleState[] swerveModuleStates = currentSetpoint.moduleStates();
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxModuleSpeed);

        for (SwerveModule mod : modules) {
            mod.runSetpoint(swerveModuleStates[mod.index]);
        }
        Logger.recordOutput("Swerve/desiredModuleStates", swerveModuleStates);
        Logger.recordOutput("Swerve/ActualModuleStates", getModuleStates());
    }

    /**
     * 
     * Make the swerve drive move
     * 
     * @param targetSpeeds the desired chassis speeds
     */
    public void drive(ChassisSpeeds targetSpeeds) {
        targetSpeeds = ChassisSpeeds.discretize(targetSpeeds, Constants.loopPeriodSecs);

        lastMovingYaw = getYaw().getRadians();

        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxModuleSpeed);

        for (SwerveModule mod : modules) {
            mod.runSetpoint(swerveModuleStates[mod.index]);
        }
    }

    /**
     * Sets all of the modules to their desired states
     * 
     * @param desiredStates array of states for the modules to be set to
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxModuleSpeed);

        for (SwerveModule mod : modules) {
            mod.runSetpoint(desiredStates[mod.index]);
        }
    }


    /**
     * Gets all of the current module states
     * 
     * @return array of the current module states
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : modules) {
            states[mod.index] = mod.getState();
        }
        return states;
    }

    /**
     * Gets all of the current module positions
     * 
     * @return array of the current module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : modules) {
            positions[mod.index] = mod.getPosition();
        }
        return positions;
    }


    /**
     * Gets ths current chassis speeds
     * 
     * @return current chassis speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Gets the current pose, according to our pose estimator
     * 
     * @return current pose in meters
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets our odometry to desired pose
     * 
     * @param pose pose to set odometry to
     */
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /**
     * Sets the current gyro yaw to 0 degrees
     */
    public void zeroGyro() {
        gyroIO.setYaw(0);
        lastMovingYaw = 0;
        lastYaw = Rotation2d.fromDegrees(0);
    }

    /**
     * Sets the current gyro yaw to 180 degrees
     */
    public void reverseZeroGyro() {
        gyroIO.setYaw(180);
        lastMovingYaw = 180;
        lastYaw = Rotation2d.fromDegrees(180);
    }

    /**
     * Sets the modules to an X shape to make the robot harder to push
     */
    public void configToX() {
        modules[0].runSetpoint(new SwerveModuleState(1, new Rotation2d(Math.toRadians(45))));
        modules[1].runSetpoint(new SwerveModuleState(1, new Rotation2d(Math.toRadians(315))));
        modules[2].runSetpoint(new SwerveModuleState(1, new Rotation2d(Math.toRadians(315))));
        modules[3].runSetpoint(new SwerveModuleState(1, new Rotation2d(Math.toRadians(45))));
    }

    // boolean <- pronnounced 'bolly-un'
    // erm... what the sigma?

    /**
     * Stops the swerve drive
     */
    public void stop() {
        drive(new ChassisSpeeds());
    }

    public Command driveToPose(Pose2d pose) {

        return AutoBuilder.pathfindToPose(pose,
                new PathConstraints(Constants.SwerveConstants.maxSpeed, Constants.SwerveConstants.maxAccel,
                        Constants.SwerveConstants.maxAngularVelocity,
                        Constants.SwerveConstants.maxAngularAcceleration));
    }

    // public boolean aligned() {
    // TODO add this!
    // }

    public boolean aligned(Rotation2d angle, double toleranceDeg) {
        return Math.abs(angle.minus(getYaw()).getDegrees()) < toleranceDeg;
    }


    // public void addVisionMG2(Vision vision) {

    //     Matrix<N3, N1> stdDev;
    //     Matrix<N3, N1> truststdDev = DriverStation.isAutonomous() ? LimelightConstants.trustautostdDev
    //             : LimelightConstants.trusttelestdDev;
    //     Matrix<N3, N1> regstdDev = DriverStation.isAutonomous() ? LimelightConstants.regautostdDev
    //             : LimelightConstants.regtelestdDev;
    //     Logger.recordOutput("Pose/" + vision.getLimelightName(), vision.getBotPoseMG2());

    //     // stdDev = regstdDev;
    //     if (vision.getPoseEstimateMG2().tagCount >= 2) {
    //         stdDev = truststdDev;
    //     } else {
    //         stdDev = regstdDev;
    //     }

    //     if (vision.getPoseValidMG2(getYaw())) {
    //         poseEstimator.addVisionMeasurement(vision.getBotPoseMG2(), vision.getPoseTimestampMG2(), stdDev);
    //         // System.out.println("yes " + vision.getLimelightName() + " " +
    //         // Timer.getFPGATimestamp());
    //     }

    // }

}