package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.util.swerve.ModuleConstants;

public class Constants {
        public static final Mode currentMode = RobotBase.isSimulation() ? Mode.SIM
                        : (RobotBase.isReal() ? Mode.REAL : Mode.REPLAY);

        public static enum Mode {
                /** Running on a real robot. */
                REAL,

                /** Running a physics simulator. */
                SIM,

                /** Replaying from a log file. */
                REPLAY
        }

        public static final double loopPeriodSecs = 0.02; // 50 hz
        // public static final Integer AmpConstants = 0;

        public static final class RobotConstants {
                public static final double mass = 52.1631; // kg
                public static final double MOI = 0.25; // estimate kg m^2
        }

        public static final class VisionConstants {
                public static final Vector<N3> autoStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(1));

        }

        public static final class SwerveConstants {

                public static final double DRIVE_MOMENT_OF_INERTIA = 0;
                public static final double ANGLE_MOMENT_OF_INERTIA = 0;

                public static final String CAN_BUS = "";

                public static final int pigeonID = 0;

                public static final double maxSpeed = 4.5;
                public static final double maxModuleSpeed = 5.5;
                public static final double maxAccel = 3;
                public static final double maxModuleAngularVelocity = 4 * Math.PI; //TODO find this 
                public static final double maxModuleAngularAcceleration = Math.PI;

                public static final double stickDeadband = 0;

                public static final boolean useFOC = true;

                // AUTO ALIGNMENT ONLY !!!!!!11!!!1!1!!!
                public static final double rotationToleranceAlignment = 2.5;

                /* Drivetrain Constants */
                public static final double trackWidth = 0.521; // 20.5 in -> meters
                public static final double wheelBase = 0.521; // meters
                public static final double driveBaseRadius = Math.sqrt(2 * Math.pow(wheelBase / 2, 2));
                public static final double wheelCircumference = Units.inchesToMeters(3.897375) * Math.PI; // TODO 6328
                                                                                                          // rotational
                                                                                                          // wheel
                                                                                                          // radius
                                                                                                          // characterization
                public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);
                public static final double driveGearRatio = (5.36 / 1.0); // Bridge's wierd L3+

                /* Auto Constants */
                public static final double translationKP = 6;
                public static final double translationKI = 0.0;
                public static final double translationKD = 1.4;

                public static final double rotationKP = 4;
                public static final double rotationKI = 0.0;
                public static final double rotationKD = 0.0;

                /* Teleop Constants */

                //used for aim assist
                public static final double teleopTranslationKP = 1;
                public static final double teleopTranslationKI = 0;
                public static final double teleopTranslationKD = 0;

                //used for heading maintanence
                public static final double teleopRotationKP = 2;
                public static final double teleopRotationKI = 0.0;
                public static final double teleopRotationKD = 0.0;

                //used for autoalign
                public static final double autoAlignRotationKP = 3.5;
                public static final double autoAlignRotationKI = 0.0;
                public static final double autoAlignRotationKD = 0.0;

                /* Module Translations */
                public static final Translation2d[] moduleTranslations = new Translation2d[] {
                                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
                };

                /* Swerve Kinematics */
                public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                moduleTranslations);

                /* Angle Motor Configs */
                public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
                public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;

                public static final double SLIP_CURRENT = 120; // Amps, current where the wheels start to slip TODO find
                                                               // this
                public static final boolean angleSupplyCurrentLimitEnable = true;
                public static final double angleSupplyCurrentLimit = 25.0;
                public static final double angleSupplyCurrentThreshold = 40.0;
                public static final double angleSupplyTimeThreshold = 0.1;

                public static final boolean angleStatorCurrentLimitEnable = true;
                public static final double angleStatorCurrentLimit = 20;

                /* Drive Configs */
                public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
                public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

                public static final boolean driveSupplyCurrentLimitEnable = true;
                public static final double driveSupplyCurrentLimit = 35.0;
                public static final double driveSupplyCurrentThreshold = 60.0;
                public static final double driveSupplyTimeThreshold = 0.1;

                public static final boolean driveStatorCurrentLimitEnable = true;
                public static final double driveStatorCurrentLimit = 60; // for now, maybe 50 if were feeling spicy

                public static final double openLoopRamp = 0.25;
                public static final double closedLoopRamp = 0.0;

                /* CanCoder Configs */
                public static final SensorDirectionValue sensorDirection = SensorDirectionValue.CounterClockwise_Positive;

                /*
                 * Offsets must be done with bevels facing towards spivit motor
                 */
                /* FRONT LEFT */
                public static final class Module0Constants {
                        public static final int cancoderID = 1;
                        public static final int angleMotorID = 2;
                        public static final int driveMotorID = 1;
                        public static final Slot0Configs driveConfigs = getDriveConfigs();
                        public static final Slot0Configs angleConfigs = getAngleConfigs();
                        public static final Rotation2d angleOffset = Rotation2d.fromRadians(0);
                        public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID,
                                        cancoderID,
                                        angleOffset, driveConfigs, angleConfigs, true, true, false, false);
                }

                /* FRONT RIGHT */
                public static final class Module1Constants {
                        public static final int cancoderID = 2;
                        public static final int angleMotorID = 4;
                        public static final int driveMotorID = 3;
                        public static final Slot0Configs driveConfigs = getDriveConfigs();
                        public static final Slot0Configs angleConfigs = getAngleConfigs();

                        public static final Rotation2d angleOffset = Rotation2d.fromRadians(0);

                        public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID,
                                        cancoderID,
                                        angleOffset, driveConfigs, angleConfigs, true, true, false, false);
                }

                /* BACK LEFT */
                public static final class Module2Constants {
                        public static final int cancoderID = 3;
                        public static final int angleMotorID = 6;
                        public static final int driveMotorID = 5;
                        public static final Slot0Configs driveConfigs = getDriveConfigs();
                        public static final Slot0Configs angleConfigs = getAngleConfigs();

                        public static final Rotation2d angleOffset = Rotation2d.fromRadians(0);

                        public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID,
                                        cancoderID,
                                        angleOffset, driveConfigs, angleConfigs, true, true, false, false);
                }

                /* BACK RIGHT */
                public static final class Module3Constants {
                        public static final int cancoderID = 4;
                        public static final int angleMotorID = 8;
                        public static final int driveMotorID = 7;
                        public static final Rotation2d angleOffset = Rotation2d.fromRadians(0);

                        public static final Slot0Configs driveConfigs = getDriveConfigs();
                        public static final Slot0Configs angleConfigs = getAngleConfigs();

                        public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID,
                                        cancoderID,
                                        angleOffset, driveConfigs, angleConfigs, true, true, false, false);
                }

                public static final Vector<N3> autostateStdDevs = VecBuilder.fill(0.01, 0.01,
                                Units.degreesToRadians(1)); // formerly
                public static final Vector<N3> telestateStdDevs = VecBuilder.fill(0.15, 0.15,
                                Units.degreesToRadians(1));

                public static Slot0Configs getAngleConfigs() {
                        return new Slot0Configs()
                                        .withKP(100) // 6328 default values
                                        .withKI(0)
                                        .withKD(0.5)
                                        .withKS(0.1)
                                        .withKV(1.91)
                                        .withKA(0);
                }

                public static Slot0Configs getDriveConfigs() {
                        return new Slot0Configs()
                                        .withKP(0.1) // 6328 default values
                                        .withKI(0)
                                        .withKD(0)
                                        .withKS(0)
                                        .withKV(0.124);

                }
        }

        public static final class FieldConstants {
                public static final double fieldLength = 16.541; // meters
                public static final double fieldWidth = 8.211;
        }

}
