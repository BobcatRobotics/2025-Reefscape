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
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  // TODO make this as small as possible
  public static final Rotation2d ARM_TOLERANCE = Rotation2d.fromDegrees(2);

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

  public boolean inTolerance(SuperstructureState desiredState) {
    if (desiredState.armState == ArmState.NO_OP) {
      return true;
    }
    double rotations =
        inputs.flipped ? 0.5 - desiredState.armState.rotations : desiredState.armState.rotations;
    return Math.abs(inputs.absolutePosition.getRotations() - rotations)
        < ARM_TOLERANCE.getRotations();
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

  public ArmState getState() {
    return inputs.state;
  }

  public boolean isFlipped() {
    return inputs.flipped;
  }
}
