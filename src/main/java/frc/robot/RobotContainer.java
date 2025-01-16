// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AidenGamepads.EightBitDo;
import frc.lib.util.AidenGamepads.Ruffy;
import frc.robot.Commands.Swerve.TeleopSwerve;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Assists.AimAssist;
import frc.robot.Subsystems.Swerve.Gyro.GyroIO;
import frc.robot.Subsystems.Swerve.Gyro.GyroIOPigeon2;
import frc.robot.Subsystems.Swerve.SwerveModule.SwerveModuleIO;
import frc.robot.Subsystems.Swerve.SwerveModule.SwerveModuleIOSim;
import frc.robot.Subsystems.Swerve.SwerveModule.SwerveModuleIOTalonFX;

public class RobotContainer {

        /* Joysticks + Gamepad */
        private final Ruffy rotate = new Ruffy(1);
        private final Ruffy strafe = new Ruffy(0);
        private final EightBitDo gp = new EightBitDo(2);

        /* Subsystems */
        public final Swerve m_swerve;

        /* Commands */

        /* Shuffleboard Inputs */
        private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

        public RobotContainer() {
                switch (Constants.currentMode) {
                        // Real robot, instantiate hardware IO implementations
                        case REAL:
                                m_swerve = new Swerve(new GyroIOPigeon2(),
                                                new SwerveModuleIOTalonFX(SwerveConstants.Module0Constants.constants),
                                                new SwerveModuleIOTalonFX(SwerveConstants.Module1Constants.constants),
                                                new SwerveModuleIOTalonFX(SwerveConstants.Module2Constants.constants),
                                                new SwerveModuleIOTalonFX(SwerveConstants.Module3Constants.constants)
                                                );
                                break;
                        case SIM:
                                m_swerve = new Swerve(
                                        new GyroIOPigeon2(),
                                        new SwerveModuleIOSim(SwerveConstants.Module0Constants.constants),
                                        new SwerveModuleIOSim(SwerveConstants.Module0Constants.constants),
                                        new SwerveModuleIOSim(SwerveConstants.Module0Constants.constants),
                                        new SwerveModuleIOSim(SwerveConstants.Module0Constants.constants));
                                break;
                        // Replayed robot, disable IO implementations
                        default:
                                m_swerve = new Swerve(new GyroIO() {
                                },
                                                new SwerveModuleIO() {
                                                },
                                                new SwerveModuleIO() {
                                                },
                                                new SwerveModuleIO() {
                                                },
                                                new SwerveModuleIO() {
                                                });
                                break;

                }

                configureBindings();
        }

        public boolean autoChooserInitialized(){
                return autoChooser.get() != null;
        }

        /**
         * this should only be called once DS and FMS are attached
         */
        public void configureAutos(){
                
                /*
                 * Auto Chooser
                 * 
                 * Names must match what is in PathPlanner
                 * Please give descriptive names
                 */
                autoChooser.addDefaultOption("Do Nothing", Commands.none());
                autoChooser.addDefaultOption("Test", new PathPlannerAuto("test"));        
        }
                

        /**
         * IMPORTANT NOTE:
         * When a gamepad value is needed by a command, don't
         * pass the gamepad to the command, instead have the
         * constructor for the command take an argument that
         * is a supplier of the value that is needed. To supply
         * the values, use an anonymous function like this:
         * 
         * () -> buttonOrAxisValue
         */
        public void configureBindings() {

                AimAssist assist = new AimAssist(
                        () -> new Translation2d(),
                        () -> new Rotation2d(),
                        gp.a,
                        gp.b,
                        SwerveConstants.teleopTranslationKP,
                        SwerveConstants.teleopTranslationKI,
                        SwerveConstants.teleopTranslationKD, 
                        SwerveConstants.autoAlignRotationKP,
                        SwerveConstants.autoAlignRotationKI,
                        SwerveConstants.autoAlignRotationKD);
               
                /* Drive with joysticks */
                m_swerve.setDefaultCommand(
                                new TeleopSwerve(
                                        m_swerve,
                                        gp.leftXAxis,
                                        gp.leftYAxis,
                                        gp.rightXAxis,
                                        () -> false,
                                        () -> 0,
                                        () -> 0,
                                        assist));
                }

        public Command getAutonomousCommand() {
                return autoChooser.get();
        }
}
