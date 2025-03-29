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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.AidensGamepads.ButtonBoard;
import frc.robot.AidensGamepads.LogitechJoystick;
import frc.robot.AidensGamepads.Ruffy;
import frc.robot.Constants.Constants;
import frc.robot.Constants.TunerConstants25;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.CharacterizationCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SuperstructureActions;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIO;
import frc.robot.subsystems.Climber.ClimberIOTalonFX;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.CoralIntake.CoralIntakeIO;
import frc.robot.subsystems.CoralIntake.CoralIntakeIOSim;
import frc.robot.subsystems.CoralIntake.CoralIntakeIOTalonFX;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.GyroIO;
import frc.robot.subsystems.Drive.GyroIOPigeon2;
import frc.robot.subsystems.Drive.SwerveModuleIO;
import frc.robot.subsystems.Drive.SwerveModuleIOSim;
import frc.robot.subsystems.Drive.SwerveModuleIOTalonFX;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorIO;
import frc.robot.subsystems.EndEffector.EndEffectorIOTalonFX;
import frc.robot.subsystems.Limelight.Vision;
import frc.robot.subsystems.Limelight.VisionIO;
import frc.robot.subsystems.Limelight.VisionIOLimelight;
import frc.robot.subsystems.PhotonVision.Photon;
import frc.robot.subsystems.PhotonVision.PhotonIO;
import frc.robot.subsystems.PhotonVision.PhotonIOPhoton;
import frc.robot.subsystems.Superstructure.Arm.Arm;
import frc.robot.subsystems.Superstructure.Arm.ArmIO;
import frc.robot.subsystems.Superstructure.Arm.ArmIOSim;
import frc.robot.subsystems.Superstructure.Arm.ArmIOTalonFX;
import frc.robot.subsystems.Superstructure.Elevator.Elevator;
import frc.robot.subsystems.Superstructure.Elevator.ElevatorIO;
import frc.robot.subsystems.Superstructure.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Superstructure.Elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.Superstructure.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.util.Enums.BranchSide;
import frc.robot.util.Enums.ScoringLevel;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
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
  public static final int END_EFFECTOR_LASER_ID = 2;
  // intake ids
  public static final int CARWASH_TALON_ID = 20;
  public static final int INTAKE_ROLLER_TALON_ID = 23;
  public static final int INTAKE_PIVOT_TALON_ID = 27;
  public static final int INTAKE_LASER_ID = 1;
  // climber ids
  public static final int CLIMBER_TALON_ID = 18; // TODO find this

  // Subsystems
  public final Vision limelightfl;
  public final Vision limelightfr;
  public final Vision limelightbl;
  public final Vision limelightbr;

  public double stickInvert = 1;

  final Drive drive;
  public Photon photon;
  // public final Vision limelight;
  private final Superstructure superstructure;
  private final EndEffector endEffector;
  private final CoralIntake intake;
  private final Climber climber;

  // Controllers
  // driver
  private final Ruffy leftRuffy = new Ruffy(0);
  private final Ruffy rightRuffy = new Ruffy(1);

  // operator
  private final LogitechJoystick joystick = new LogitechJoystick(2);
  private final ButtonBoard buttonBoard = new ButtonBoard(3);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final Alert buttonBoardUnpluggedAlert =
      new Alert("Button board unplugged!", AlertType.kWarning);
  private final Alert joystickUnpluggedAlert =
      new Alert("Operator Joystick unplugged!", AlertType.kWarning);
  private final Alert leftRuffyUnpluggedAlert =
      new Alert("Left ruffy unplugged!", AlertType.kWarning);
  private final Alert rightRuffyUnpluggedAlert =
      new Alert("Right Ruffy unplugged!", AlertType.kWarning);

  public Supplier<Angle> trimSupplier = () -> Rotations.of(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new SwerveModuleIOTalonFX(TunerConstants25.FrontLeft),
                new SwerveModuleIOTalonFX(TunerConstants25.FrontRight),
                new SwerveModuleIOTalonFX(TunerConstants25.BackLeft),
                new SwerveModuleIOTalonFX(TunerConstants25.BackRight));
        // drive =
        //     new Drive(
        //         new GyroIO() {},
        //         new SwerveModuleIO() {},
        //         new SwerveModuleIO() {},
        //         new SwerveModuleIO() {},
        //         new SwerveModuleIO() {});

        limelightfl =
            new Vision(drive, new VisionIOLimelight(Constants.LimelightFLConstants.constants));
        limelightfr =
            new Vision(drive, new VisionIOLimelight(Constants.LimelightFRConstants.constants));
        limelightbl =
            new Vision(drive, new VisionIOLimelight(Constants.LimelightBLConstants.constants));
        limelightbr =
            new Vision(drive, new VisionIOLimelight(Constants.LimelightBRConstants.constants));

        photon = new Photon(new PhotonIOPhoton("arducam"), "arducam");
        // photon = new Photon(new PhotonIO() {});
        // limelightfl = new Vision(drive, new VisionIO() {});
        // limelightfr = new Vision(drive, new VisionIO() {});
        // limelightbl = new Vision(drive, new VisionIO() {});
        // limelightbr = new Vision(drive, new VisionIO() {});

        climber = new Climber(new ClimberIOTalonFX(CLIMBER_TALON_ID));

        // superstructure =
        // new Superstructure(
        // new Arm(new ArmIO() {}), // new ArmIOTalonFX(ARM_TALON_ID, ARM_ENCODER_ID)),
        // new Elevator(new ElevatorIOTalonFX(ELEVATOR_TALON_ID, ELEVATOR_ENCODER_ID)));

        // drive =
        // new Drive(
        // new GyroIO() {},
        // new SwerveModuleIO() {},
        // new SwerveModuleIO() {},
        // new SwerveModuleIO() {},
        // new SwerveModuleIO() {});
        // limelight = new Vision(drive, new VisionIO() {});
        // superstructure =
        //     new Superstructure(new Arm(new ArmIO() {}), new Elevator(new ElevatorIO() {}));

        superstructure =
            new Superstructure(
                new Arm(new ArmIOTalonFX(ARM_TALON_ID, ARM_ENCODER_ID)),
                new Elevator(new ElevatorIOTalonFX(ELEVATOR_TALON_ID, ELEVATOR_ENCODER_ID)));

        endEffector =
            new EndEffector(new EndEffectorIOTalonFX(END_EFFECTOR_TALON_ID, END_EFFECTOR_LASER_ID));
        // endEffector = new EndEffector(new EndEffectorIO() {});
        intake =
            new CoralIntake(
                new CoralIntakeIOTalonFX(
                    INTAKE_ROLLER_TALON_ID, INTAKE_PIVOT_TALON_ID, INTAKE_LASER_ID));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new SwerveModuleIOSim(TunerConstants25.FrontLeft),
                new SwerveModuleIOSim(TunerConstants25.FrontRight),
                new SwerveModuleIOSim(TunerConstants25.BackLeft),
                new SwerveModuleIOSim(TunerConstants25.BackRight));
        superstructure =
            new Superstructure(new Arm(new ArmIOSim()), new Elevator(new ElevatorIOSim()));
        endEffector = new EndEffector(new EndEffectorIO() {});

        limelightfl = new Vision(drive, new VisionIO() {});
        limelightfr = new Vision(drive, new VisionIO() {});
        limelightbl = new Vision(drive, new VisionIO() {});
        limelightbr = new Vision(drive, new VisionIO() {});
        photon = new Photon(new PhotonIO() {}, "Sim");
        intake = new CoralIntake(new CoralIntakeIOSim() {});
        climber = new Climber(new ClimberIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {});
        intake = new CoralIntake(new CoralIntakeIO() {});
        superstructure =
            new Superstructure(new Arm(new ArmIO() {}), new Elevator(new ElevatorIO() {}));

        limelightfl = new Vision(drive, new VisionIO() {});
        limelightfr = new Vision(drive, new VisionIO() {});
        limelightbl = new Vision(drive, new VisionIO() {});
        limelightbr = new Vision(drive, new VisionIO() {});

        endEffector = new EndEffector(new EndEffectorIO() {});
        photon = new Photon(new PhotonIO() {}, "Sim");
        climber = new Climber(new ClimberIO() {});

        break;
    }

    // Set up auto routines
    registerCommands();
    // autobuilder handles 'do nothing' command
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // autoChooser.addOption("Popsicle", new PathPlannerAuto("Popsicle"));
    autoChooser.addOption(
        "MVP CW L4",
        AutoCommands.fullAutoReefScore(
            drive, superstructure, endEffector, BranchSide.CLOCKWISE, ScoringLevel.CORAL_L4, true));
    autoChooser.addOption(
        "SwerveFFTest", CharacterizationCommands.velocityRampTuning(drive, Seconds.of(1.5)));

    // Set up SysId routine
    // drivetrain
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        CharacterizationCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization",
        CharacterizationCommands.feedforwardCharacterization(drive));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void registerCommands() {

    // the naming convention is "Action" "Piece" "Level"

    NamedCommands.registerCommand(
        "Prep Coral L1",
        superstructure.setState(SuperstructureState.CORAL_PREP_L1, endEffector::hasPiece));
    NamedCommands.registerCommand(
        "Score Coral L1",
        new ParallelCommandGroup(
            superstructure.setState(SuperstructureState.CORAL_SCORE_L1, endEffector::hasPiece),
            endEffector.outtakeCommand().until(() -> !endEffector.hasPiece())));

    NamedCommands.registerCommand("stopOverride", Commands.run(() -> drive.clearPPOverride()));
    NamedCommands.registerCommand("AutoalignCCW", DriveCommands.driveToReefAuto(drive, true));
    NamedCommands.registerCommand("AutoalignCW", DriveCommands.driveToReefAuto(drive, false));
    NamedCommands.registerCommand(
        "PopsicleLick",
        superstructure
            .setState(SuperstructureState.POPSICLE_LICK, endEffector::hasPiece)
            .alongWith(endEffector.intakeCoralCommand())
            .until(endEffector::hasPiece)
            .andThen(
                superstructure
                    .setState(SuperstructureState.RIGHT_SIDE_UP_IDLE, endEffector::hasPiece)
                    .alongWith(endEffector.idleCoralCommand())));

    // retract and start intaking
    NamedCommands.registerCommand(
        "RetractThatThang",
        superstructure
            .setState(SuperstructureState.UPSIDE_DOWN_IDLE, endEffector::hasPiece)
            .deadlineFor(endEffector.coralOut(() -> ScoringLevel.CORAL_L4))
            .alongWith(
                Commands.run(
                        () -> {
                          intake.deploy();
                          intake.runIn();
                        },
                        intake)
                    .until(intake::hasPiece)));

    NamedCommands.registerCommand(
        "HandoffThenPrep", SuperstructureActions.handoffThenPrepL4(superstructure, endEffector));

    NamedCommands.registerCommand(
        "ScoreCoralL4CCW",
        AutoCommands.fullAutoReefScore(
            drive,
            superstructure,
            endEffector,
            BranchSide.COUNTER_CLOCKWISE,
            ScoringLevel.CORAL_L4,
            true));
    NamedCommands.registerCommand(
        "ScoreCoralL4CW",
        AutoCommands.fullAutoReefScore(
            drive, superstructure, endEffector, BranchSide.CLOCKWISE, ScoringLevel.CORAL_L4, true));

    NamedCommands.registerCommand(
        "ScoreCoralL4CCWOverride",
        AutoCommands.fullAutoReefScoreOverride(
            drive,
            superstructure,
            endEffector,
            BranchSide.COUNTER_CLOCKWISE,
            ScoringLevel.CORAL_L4));
    NamedCommands.registerCommand(
        "ScoreCoralL4CWOverride",
        AutoCommands.fullAutoReefScoreOverride(
            drive, superstructure, endEffector, BranchSide.CLOCKWISE, ScoringLevel.CORAL_L4));

    NamedCommands.registerCommand(
        "StowSuperstructureThenFlip",
        superstructure
            .setState(SuperstructureState.RIGHT_SIDE_UP_IDLE, endEffector::hasPiece)
            .andThen(
                superstructure.setState(
                    SuperstructureState.UPSIDE_DOWN_IDLE, endEffector::hasPiece)));

    NamedCommands.registerCommand(
        "hpWaitThenIntake",
        superstructure
            .setState(SuperstructureState.UPSIDE_DOWN_IDLE, endEffector::hasPiece)
            .alongWith(
                new InstantCommand(
                    () -> {
                      intake.deploy();
                      intake.runIn(); // deploy and run in for one second
                    }))
            .withTimeout(5)
            // flippy flip flip
            .andThen(
                new RunCommand(() -> intake.retract()).withTimeout(1)
                // .andThen(
                //     new RunCommand(
                //             () -> { // retract for a quarter second
                //               intake.retract();
                //             })
                //         .withTimeout(0.25))
                // .andThen(new RunCommand(() -> intake.deploy()).withTimeout(0.25)) // depo
                ));

    NamedCommands.registerCommand(
        "PrepL4", superstructure.goToPrepPos(ScoringLevel.CORAL_L4, () -> false));
    NamedCommands.registerCommand(
        "PlaceL4",
        superstructure
            .setState(SuperstructureState.CORAL_SCORE_L4, endEffector::hasPiece)
            .andThen(
                superstructure.setState(
                    SuperstructureState.POST_CORAL_SCORE_L4, endEffector::hasPiece)));
    NamedCommands.registerCommand(
        "FlipAndIntake",
        new RunCommand(() -> intake.deploy())
            .withTimeout(0.25)
            .andThen(
                new RunCommand(
                        () -> {
                          intake.retract();
                          intake.setSpeed(Amps.of(30));
                        })
                    .withTimeout(0.25))
            .andThen(new RunCommand(() -> intake.deploy()).withTimeout(0.25))
            .andThen(new RunCommand(() -> intake.retract()).withTimeout(0.25))
            .andThen(new RunCommand(() -> intake.deploy()).withTimeout(0.25))
            .andThen(
                new RunCommand(
                    () -> {
                      intake.retract();
                      intake.stop();
                    }))
            .handleInterrupt(() -> intake.retract()));

    NamedCommands.registerCommand(
        "DeployTounge",
        new RunCommand(() -> intake.deploy())
            .withTimeout(0.25)
            .andThen(
                new RunCommand(
                        () -> {
                          intake.retract();
                          intake.setSpeed(Amps.of(30));
                        })
                    .withTimeout(0.25))
            .andThen(new RunCommand(() -> intake.deploy()).withTimeout(0.25))
            .andThen(new RunCommand(() -> intake.retract()).withTimeout(0.25))
            .andThen(new RunCommand(() -> intake.deploy()).withTimeout(0.25))
            .andThen(
                new RunCommand(
                    () -> {
                      intake.retract();
                      intake.stop();
                    }))
            .handleInterrupt(() -> intake.retract()));
    // .andThen(SuperstructureActions.handoff(superstructure, endEffector)));
    NamedCommands.registerCommand(
        "Handoff", SuperstructureActions.handoffNoIdle(superstructure, endEffector));

    NamedCommands.registerCommand("StopEEDefault", Commands.runOnce(() -> endEffector.setSpeed(0)));

    NamedCommands.registerCommand(
        "DriveToCoral",
        DriveCommands.driveToCoral(
            drive, photon, () -> 0, () -> 0, () -> 0, superstructure::getElevatorPercentage));

    NamedCommands.registerCommand(
        "retractIntake",
        new InstantCommand(
            () -> {
              intake.retract();
            }));
    NamedCommands.registerCommand(
        "Stow",
        superstructure.setState(SuperstructureState.RIGHT_SIDE_UP_IDLE, endEffector::hasPiece));

    NamedCommands.registerCommand(
        "StowIntake",
        new InstantCommand(
            () -> {
              intake.retract();
            }));
    NamedCommands.registerCommand(
        "DeployIntakeEndDeployed",
        new RunCommand(() -> intake.deploy())
            .alongWith(
                new RunCommand(
                    () -> {
                      intake.setSpeed(Amps.of(-30));
                    })));

    NamedCommands.registerCommand(
        "PrepAlgaePluckL2",
        superstructure
            .setState(SuperstructureState.ALGAE_PREP_L2, endEffector::hasPiece)
            .alongWith(endEffector.intakeAlgaeCommand())
            .until(endEffector::hasPiece));

    NamedCommands.registerCommand(
        "PrepAlgaePluckL3",
        SuperstructureActions.prepScore(
                ScoringLevel.ALGAE_L3, drive::isCoralSideDesired, superstructure, endEffector)
            .alongWith(endEffector.intakeAlgaeCommand())
            .until(endEffector::hasPiece));

    NamedCommands.registerCommand(
        "ScoreAlgae",
        superstructure
            .setState(SuperstructureState.NET_SCORE, endEffector::hasPiece)
            .andThen(endEffector.outtakeFastCommand().until(() -> !endEffector.hasPiece())));
    NamedCommands.registerCommand(
        "PrepNetScore",
        SuperstructureActions.prepScore(
                ScoringLevel.NET, drive::isCoralSideDesired, superstructure, endEffector)
            .alongWith(endEffector.intakeAlgaeCommand()));
  }

  public void updateControllerAlerts() {
    buttonBoardUnpluggedAlert.set(!buttonBoard.isConnected());
    joystickUnpluggedAlert.set(!joystick.isConnected());
    leftRuffyUnpluggedAlert.set(!leftRuffy.isConnected());
    rightRuffyUnpluggedAlert.set(!rightRuffy.isConnected());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /* driver controls */

    // TODO decrease speed when CoG really high
    // default commands
    drive.setDefaultCommand(
        DriveCommands.fieldRelativeJoystickDrive(
            drive,
            () -> -leftRuffy.yAxis.getAsDouble() * stickInvert,
            () -> leftRuffy.xAxis.getAsDouble() * stickInvert,
            () -> -rightRuffy.xAxis.getAsDouble(),
            superstructure::getElevatorPercentage,
            rightRuffy::getZ,
            leftRuffy::getZ));

    // Reset gyro to 0 deg
    rightRuffy.button.onTrue(
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive)
            .andThen(
                Commands.runOnce(
                    () -> {
                      // limelightbl.resetGyroLL4();
                      // limelightbr.resetGyroLL4();
                    }))
            .ignoringDisable(true));

    leftRuffy.button.onTrue(
        Commands.runOnce(() -> stickInvert = stickInvert * -1).ignoringDisable(true));

    // leftRuffy.button.onTrue(
    // AutoCommands.fullAutoReefScore(
    // drive,
    // superstructure,
    // endEffector,
    // BranchSide.COUNTER_CLOCKWISE,
    // ScoringLevel.CORAL_L4));

    // leftRuffy.button.whileTrue(
    //     DriveCommands.driveToCoral(
    //         drive, photon, () -> 0, () -> 0, () -> 0, superstructure::getElevatorPercentage));
    // .withDeadline(
    // SuperstructureActions.intakeCoralGround(superstructure, intake,
    // trimSupplier)));

    /* operator controls */

    // default commands
    endEffector.setDefaultCommand(endEffector.idleCoralCommand());
    climber.setDefaultCommand(climber.idleIn());

    double aidenAlignStrength = 1;
    // autoalign
    joystick.trigger.whileTrue(
        DriveCommands.driveToReef(
            drive,
            () -> -leftRuffy.yAxis.getAsDouble(),
            () -> leftRuffy.xAxis.getAsDouble(),
            () -> -rightRuffy.xAxis.getAsDouble(),
            joystick.povRight(),
            joystick.povLeft(),
            () -> -joystick.yAxis.getAsDouble() * aidenAlignStrength,
            () -> -joystick.xAxis.getAsDouble() * aidenAlignStrength));

    // reef levels
    buttonBoard.l1.onTrue(
        SuperstructureActions.prepScore(
            ScoringLevel.CORAL_L1, drive::isCoralSideDesired, superstructure, endEffector));

    buttonBoard.l2.onTrue(
        new ConditionalCommand(
            SuperstructureActions.prepScore(
                    ScoringLevel.ALGAE_L2, drive::isCoralSideDesired, superstructure, endEffector)
                .alongWith(endEffector.intakeAlgaeCommand()),
            SuperstructureActions.prepScore(
                ScoringLevel.CORAL_L2, drive::isCoralSideDesired, superstructure, endEffector),
            this::shouldUseAlgae));

    buttonBoard.l3.onTrue(
        new ConditionalCommand(
            SuperstructureActions.prepScore(
                    ScoringLevel.ALGAE_L3, drive::isCoralSideDesired, superstructure, endEffector)
                .alongWith(endEffector.intakeAlgaeCommand()),
            SuperstructureActions.prepScore(
                ScoringLevel.CORAL_L3, drive::isCoralSideDesired, superstructure, endEffector),
            this::shouldUseAlgae));

    buttonBoard.l4.onTrue(
        SuperstructureActions.prepScore(
                ScoringLevel.CORAL_L4, drive::isCoralSideDesired, superstructure, endEffector)
            .alongWith(endEffector.idleCoralCommand()));

    buttonBoard.net.onTrue(
        SuperstructureActions.prepScore(
                ScoringLevel.NET, drive::isCoralSideDesired, superstructure, endEffector)
            .alongWith(endEffector.intakeAlgaeCommand()));

    // score
    joystick.thumb.onTrue(
        new ConditionalCommand(
            superstructure.gotToLastPrepPosition(endEffector::hasPiece),
            superstructure
                .score(drive::isCoralSideDesired, endEffector::hasPiece)
                .alongWith(
                    new ConditionalCommand(
                        endEffector.stop(),
                        Commands.none(),
                        superstructure::isScoringLevelCoralL2orL3)),
            superstructure::isScoring));

    // stow
    joystick
        .povDown()
        .onTrue(
            new ConditionalCommand(
                superstructure
                    .setState(SuperstructureState.RIGHT_SIDE_UP_IDLE, endEffector::hasPiece)
                    .deadlineFor(endEffector.scoreCommand(superstructure::getState))
                    .andThen(endEffector.idleCoralCommand().unless(superstructure::isInPrepState)),
                superstructure
                    .setState(SuperstructureState.POST_CORAL_SCORE_L4, endEffector::hasPiece)
                    .deadlineFor(endEffector.scoreCommand(superstructure::getState))
                    .andThen(
                        superstructure.setState(
                            SuperstructureState.RIGHT_SIDE_UP_IDLE, endEffector::hasPiece))
                    .andThen(endEffector.idleCoralCommand().unless(superstructure::isInPrepState)),
                () -> superstructure.getState() != SuperstructureState.CORAL_SCORE_L4));

    joystick
        .povUp()
        .onTrue(
            superstructure
                .setState(SuperstructureState.UPSIDE_DOWN_IDLE, endEffector::hasPiece)
                .finallyDo(() -> endEffector.idleCoral()));

    // intake from ground
    joystick.bottomLeft.onTrue(
        superstructure.setState(SuperstructureState.UPSIDE_DOWN_IDLE, endEffector::hasPiece));

    joystick.bottomLeft.whileTrue(
        SuperstructureActions.intakeCoralGround(superstructure, intake, trimSupplier));
    // handoff
    joystick.bottomRight.onTrue(
        SuperstructureActions.handoff(superstructure, endEffector)
            .alongWith(new InstantCommand(() -> intake.setSpeed(Amps.of(10)))));

    // intake algae from ground
    joystick.topLeft.onTrue(SuperstructureActions.intakeAlgaeGround(superstructure, endEffector));

    // score algae into the processor
    joystick.topRight.onTrue(
        superstructure.setState(SuperstructureState.ALGAE_SCORE_PROCESSOR, endEffector::hasPiece));

    // zero intake
    joystick.bottom8.onTrue(new InstantCommand(() -> intake.zeroPosition()).ignoringDisable(true));

    // climber
    joystick.bottom7.whileTrue(
        new RunCommand(
                () -> {
                  climber.setDutyCycle(-joystick.getY()); // TODO maybe invert
                },
                climber)
            .alongWith(superstructure.setState(SuperstructureState.CLIMB, endEffector::hasPiece)));

    // death stars
    joystick.bottom9.whileTrue(
        new RunCommand(
            () -> {
              intake.setSpeed(Amps.of(20));
            },
            intake));

    // outtake from coral intake
    joystick
        .bottom10
        .whileTrue(
            SuperstructureActions.outtakeCoralGround(superstructure, intake, trimSupplier)
                .alongWith(endEffector.outtakeCommand()))
        .onFalse(endEffector.idleCoralCommand());

    // manual override
    joystick
        .bottom12
        .whileTrue(
            superstructure.manualOverride(
                () -> joystick.zAxis.getAsDouble() * 0.15,
                () -> -joystick.yAxis.getAsDouble() * 0.4) // TODO FIX
            )
        .onFalse(superstructure.manualOverride(() -> 0, () -> 0));

    // rightRuffy
    // .axisGreaterThan(1, .5)
    // .whileTrue(
    // DriveCommands.driveToBarge(
    // drive,
    // () -> leftRuffy.yAxis.getAsDouble(),
    // () -> -leftRuffy.xAxis.getAsDouble(),
    // () -> -rightRuffy.xAxis.getAsDouble(),
    // () -> -joystick.yAxis.getAsDouble() * aidenAlignStrength,
    // () -> -joystick.xAxis.getAsDouble() * aidenAlignStrength));
    // }

    // rightRuffy
    // .axisGreaterThan(1, .5)
    // .whileTrue(
    // DriveCommands.driveToProcessor(
    // drive,
    // () -> leftRuffy.yAxis.getAsDouble(),
    // () -> -leftRuffy.xAxis.getAsDouble(),
    // () -> -rightRuffy.xAxis.getAsDouble(),
    // () -> -joystick.yAxis.getAsDouble() * aidenAlignStrength,
    // () -> -joystick.xAxis.getAsDouble() * aidenAlignStrength));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  @AutoLogOutput(key = "Superstructure/usingAlgae")
  public boolean shouldUseAlgae() {
    return drive.getAdjustY() == 0 || joystick.bottom11.getAsBoolean();
  }
}
