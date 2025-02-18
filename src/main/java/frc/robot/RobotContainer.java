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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.AidensGamepads.ButtonBoard;
import frc.robot.AidensGamepads.LogitechJoystick;
import frc.robot.AidensGamepads.Ruffy;
import frc.robot.Constants.Constants;
import frc.robot.Constants.TunerConstants25;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.GyroIO;
import frc.robot.subsystems.Drive.GyroIOPigeon2;
import frc.robot.subsystems.Drive.ModuleIO;
import frc.robot.subsystems.Drive.ModuleIOSim;
import frc.robot.subsystems.Drive.ModuleIOTalonFX;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorIO;
import frc.robot.subsystems.Limelight.Vision;
import frc.robot.subsystems.Limelight.VisionIO;
import frc.robot.subsystems.Superstructure.Arm.Arm;
import frc.robot.subsystems.Superstructure.Arm.ArmIO;
import frc.robot.subsystems.Superstructure.Arm.ArmIOTalonFX;
import frc.robot.subsystems.Superstructure.Elevator.Elevator;
import frc.robot.subsystems.Superstructure.Elevator.ElevatorIO;
import frc.robot.subsystems.Superstructure.Elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.Superstructure.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // elevator ids
  public static final int ELEVATOR_TALON_ID = 1;
  public static final int ELEVATOR_ENCODER_ID = 5;
  // arm ids
  public static final int ARM_TALON_ID = 9;
  public static final int ARM_ENCODER_ID = 6;
  // end effector ids
  public static final int END_EFFECTOR_TALON_ID = 2;
  public static final int END_EFFECTOR_LASER_ID = 1;

  // Subsystems
  public final Vision limelight;
  final Drive drive;
  // public final Vision limelight;
  private final Superstructure superstructure;
  private final Elevator elevator; // do NOT use these directly!
  private final Arm arm;
  private final EndEffector endEffector;

  // Controllers
  // driver
  private final Ruffy leftRuffy = new Ruffy(0);
  private final Ruffy rightRuffy = new Ruffy(1);
  // operator
  private final LogitechJoystick joystick = new LogitechJoystick(2);
  private final ButtonBoard buttonBoard = new ButtonBoard(3);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants25.FrontLeft),
                new ModuleIOTalonFX(TunerConstants25.FrontRight),
                new ModuleIOTalonFX(TunerConstants25.BackLeft),
                new ModuleIOTalonFX(TunerConstants25.BackRight));
        limelight =
            new Vision(
                drive, new VisionIO() {}); // new VisionIOLimelight(LimelightConstants.constants));

        arm = new Arm(new ArmIOTalonFX(ARM_TALON_ID, ARM_ENCODER_ID));
        elevator = new Elevator(new ElevatorIOTalonFX(ELEVATOR_TALON_ID, ELEVATOR_ENCODER_ID));

        superstructure = new Superstructure(arm, elevator);

        endEffector =
            new EndEffector(
                new EndEffectorIO() {}); // new EndEffectorIOTalonFX(END_EFFECTOR_TALON_ID,
        // END_EFFECTOR_LASER_ID));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants25.FrontLeft),
                new ModuleIOSim(TunerConstants25.FrontRight),
                new ModuleIOSim(TunerConstants25.BackLeft),
                new ModuleIOSim(TunerConstants25.BackRight));
        limelight = new Vision(drive, new VisionIO() {});
        arm = new Arm(new ArmIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        superstructure = new Superstructure(arm, elevator);
        endEffector = new EndEffector(new EndEffectorIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        limelight = new Vision(drive, new VisionIO() {});
        arm = new Arm(new ArmIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        superstructure = new Superstructure(arm, elevator);
        endEffector = new EndEffector(new EndEffectorIO() {});
        break;
    }

    SysIdRoutine elevatorRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> elevator.runVoltage(voltage),
                null,
                elevator // No log consumer, since data is recorded by AdvantageKit
                ));

    SysIdRoutine armRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> arm.runVoltage(voltage),
                null,
                arm // No log consumer, since data is recorded by AdvantageKit
                ));

    // Set up auto routines
    // autobuilder handles 'do nothing' command
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // drivetrain
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization",
    //     CharacterizationCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization",
    //     CharacterizationCommands.feedforwardCharacterization(drive));

    autoChooser.addOption("test auto", new PathPlannerAuto("Example Auto"));

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

    // driver controls

    // TODO decrease speed when CoG really high?
    // default commands
    drive.setDefaultCommand(
        DriveCommands.fieldRelativeJoystickDrive(
            drive, leftRuffy.xAxis, () -> -leftRuffy.yAxis.getAsDouble(), rightRuffy.xAxis));

    // Reset gyro to 0 deg
    rightRuffy.button.onTrue(
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive)
            .alongWith(Commands.runOnce(() -> limelight.resetGyroLL4(drive)))
            .ignoringDisable(true));

    // Switch to X pattern when X button is pressed
    leftRuffy.button.onTrue(Commands.runOnce(drive::stopWithX, drive));

    // operator controls

    // default commands
    endEffector.setDefaultCommand(endEffector.idleCommand());

    // outtake
    joystick.thumb.whileTrue(endEffector.outtakeCommand());

    // intake and go to hp pos
    joystick.bottomLeft.whileTrue(
        endEffector
            .intakeCommand()
            .alongWith(superstructure.setState(SuperstructureState.HP_INTAKE)));

    // autoalign
    joystick.trigger.whileTrue(
        DriveCommands.driveToReef(
            drive,
            leftRuffy.xAxis,
            leftRuffy.yAxis,
            rightRuffy.xAxis,
            joystick.povRight(),
            joystick.povLeft()));

    // reef levels
    buttonBoard.l1.onTrue(superstructure.setState(SuperstructureState.CORAL_PREP_L1));

    buttonBoard.l2.onTrue(superstructure.setState(SuperstructureState.CORAL_PREP_L2));

    buttonBoard.l3.onTrue(superstructure.setState(SuperstructureState.CORAL_PREP_L3));

    buttonBoard.l4.onTrue(superstructure.setState(SuperstructureState.CORAL_PREP_L4));

    buttonBoard.net.onTrue(superstructure.setState(SuperstructureState.NET_SCORE));

    // score
    joystick.thumb.whileTrue(getOuttakeCommand());

    // stow
    joystick.povDown().onTrue(superstructure.setState(SuperstructureState.RIGHT_SIDE_UP_IDLE));
  }
  ;

  public ParallelCommandGroup getOuttakeCommand() {
    ParallelCommandGroup group = new ParallelCommandGroup();

    // go to the scoring location if were at a prep position
    switch (superstructure.getState()) {
      case CORAL_PREP_L1:
        group.addCommands(superstructure.setState(SuperstructureState.CORAL_SCORE_L1));
        break;
      case CORAL_PREP_L2:
        group.addCommands(superstructure.setState(SuperstructureState.CORAL_SCORE_L2));
        break;
      case CORAL_PREP_L3:
        group.addCommands(superstructure.setState(SuperstructureState.CORAL_SCORE_L3));
        break;
      case CORAL_PREP_L4:
        group.addCommands(superstructure.setState(SuperstructureState.CORAL_SCORE_L4));
        break;
      default:
        break;
    }

    // outtake untill button is released
    group.addCommands(endEffector.outtakeCommand());

    return group;
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
