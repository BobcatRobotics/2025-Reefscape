// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.RobotVisualizer;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  // TODO make this as small as possible
  public static final Rotation2d ARM_TOLERANCE = Rotation2d.fromDegrees(3.5);

  private Alert motorDisconnected = new Alert("Arm motor disconnected!", AlertType.kWarning);
  private Alert encoderDisconnected = new Alert("Arm motor disconnected!", AlertType.kWarning);

  ArmIO io;
  ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private ArmState desiredState = ArmState.RIGHT_SIDE_UP;

  private RobotVisualizer visualizer = RobotVisualizer.getInstance();

  /** Creates a new Arm. */
  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    motorDisconnected.set(!inputs.motorConnected);
    encoderDisconnected.set(!inputs.encoderConnected);
    visualizer.setArmRotation(inputs.absolutePosition);
  }

  public Rotation2d getPos() {
    return inputs.absolutePosition;
  }

  public boolean inTolerance() {
    return inputs.aligned;
  }

  @AutoLogOutput(key = "Superstructure/ArmInTolerance")
  public boolean inTolerance(SuperstructureState desiredState) {
    switch (desiredState.armState) {
      case NO_OP:
        return true;
      case UNKOWN:
        return true;
      case INTAKE_SAFE_ZONE:
        return inputs.absolutePosition.getRotations()
            < (ArmState.INTAKE_SAFE_ZONE.rotations + 10.0 / 360);
      case HANDOFF_FLIP_SAFE_ZONE:
        return inputs.absolutePosition.getRotations()
            > (ArmState.HANDOFF_FLIP_SAFE_ZONE.rotations - 10.0 / 360);
      case CORAL_SCORE_L4:
        return inputs.absolutePosition.getRotations()
            > ArmState.CORAL_SCORE_L4.rotations - 3.0 / 360;
      case POST_CORAL_SCORE_L4:
        return inputs.absolutePosition.getRotations()
            > ArmState.POST_CORAL_SCORE_L4.rotations - 3.0 / 360;
      case CORAL_PREP_L4:
        return inputs.absolutePosition.getRotations()
            > ArmState.CORAL_PREP_L4.rotations - 3.0 / 360;
      case CORAL_SCORE_L2:
        return inputs.absolutePosition.getRotations() > ArmState.CORAL_SCORE_L2.rotations - 3 / 360;
      case CORAL_SCORE_L3:
        return inputs.absolutePosition.getRotations() > ArmState.CORAL_SCORE_L3.rotations - 3 / 360;
      case NET_SCORE:
        return inputs.absolutePosition.getRotations() < ArmState.NET_SCORE.rotations + 10 / 360;
      default:
        double rotations =
            inputs.flipped
                ? 0.5 - desiredState.armState.rotations
                : desiredState.armState.rotations;
        return Math.abs(inputs.absolutePosition.getRotations() - rotations)
            < ARM_TOLERANCE.getRotations();
    }
  }

  public ArmState getDesiredState() {
    return desiredState;
  }

  public void setState(ArmState state, boolean flipped, boolean hasPiece) {
    desiredState = state;
    if (state != ArmState.NO_OP) {
      io.setDesiredState(desiredState, flipped, hasPiece);
    }
  }

  public void setManualOverride(double override) {
    io.manualOverride(override);
  }

  public ArmState getState() {
    return inputs.state;
  }

  public boolean isFlipped() {
    return inputs.flipped;
  }
}
