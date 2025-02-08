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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AidenGamepads.Ruffy;
import frc.robot.Constants.Constants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmIO;
import frc.robot.subsystems.CoralIntake.EndEffector;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Superstructure.RobotVisualizer;
import frc.robot.subsystems.Superstructure.Superstructure;



import java.util.ArrayList;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer{
   
// public RobotContainer(OI XboxController,
//                         List<LoadablePathPlannerAuto> autos,
//                         String robotName,
//                         boolean isSim,
//                         Alliance alliance,
//                         PIDConstants tranPidPathPlanner,
//                         PIDConstants rotPidPathPlanner) {
// super(driver_controller, autos, robotName, isSim, alliance, tranPidPathPlanner, rotPidPathPlanner);
//   this.isSim = isSim;
//     ArrayList<RangeSensor> distanceSensors = new ArrayList<RangeSensor>();
//       if (this.isSim) {
//           SimLC sLC = new SimLC(1, new DistanceMode(modes.SHORT), 20);
//           distanceSensors.add(sLC);
//                 distanceSensors.add(sLC);
//       } else {
//           distanceSensors.add(new LaserCan (1, new DistanceMode(modes.SHORT), 20));
//           }
//     sLC = new SpatialTOF(distanceSensors);
//     distanceSensing = new Spatial(sLC);
//        }
  // Subsystems
  // private final Drive drive;
  // public final Vision limelight;
  // private final Elevator elevator;
  private RobotVisualizer robotVisualizer = new RobotVisualizer();
  private Superstructure superstructure;
  private EndEffector m_intake = new EndEffector();

  // Controller
  private XboxController m_Controller = new XboxController(0);
  private Ruffy leftStick = new Ruffy(0);
  private Ruffy rightStick = new Ruffy(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // //drive =
        //     new Drive(
        //         new GyroIOPigeon2(),
        //         new ModuleIOTalonFX(TunerConstants24.FrontLeft),
        //         new ModuleIOTalonFX(TunerConstants24.FrontRight),
        //         new ModuleIOTalonFX(TunerConstants24.BackLeft),
        //         new ModuleIOTalonFX(TunerConstants24.BackRight));
        // elevator = new Elevator();
        // limelight = new Vision(drive, new
        // VisionIOLimelight(LimelightConstants.constants));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // drive =
        //     new Drive(
        //         new GyroIO() {},
        //         new ModuleIOSim(TunerConstants24.FrontLeft),
        //         new ModuleIOSim(TunerConstants24.FrontRight),
        //         new ModuleIOSim(TunerConstants24.BackLeft),
        //         new ModuleIOSim(TunerConstants24.BackRight));
        // limelight = new Vision(drive, new VisionIO() {});
        superstructure =
            new Superstructure(new Arm(new ArmIO() {}), new Elevator(new ElevatorIO() {}));
        break;

      default:
        // Replayed robot, disable IO implementations
        // drive =
        //     new Drive(
        //         new GyroIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {});
        // limelight = new Vision(drive, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Test", new PathPlannerAuto("Example Auto"));
    // Set up SysId routines
    // autoChooser.addOption(
    // "Drive Wheel Radius Characterization",
    // DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    // "Drive Simple FF Characterization",
    // DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Forward)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Reverse)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Forward)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Reverse)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    if (m_Controller.getRawButtonPressed(1)) {
      m_intake.run(.5);
    }

    // Default command, normal field-relative drive
    // drive.setDefaultCommand(
    //     DriveCommands.fieldRelativeJoystickDrive(
    //         drive, leftStick.yAxis, leftStick.xAxis, rightStick.xAxis));

    // rightStick.button.whileTrue(
    // DriveCommands.alignToTag(
    // drive,
    // () -> limelight.getTX().unaryMinus(),
    // () -> limelight.targetPoseCameraSpace().getX(),
    // () -> limelight.targetPoseCameraSpace().getY()));

    // Switch to X pattern when X button is pressed
    // rightStick.button.onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0 deg when B button is pressed
    // leftStick.button.onTrue(
    //     Commands.runOnce(
    //             () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new
    // Rotation2d())),
    //             drive)
    //         .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
