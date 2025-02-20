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
  // see Assets\Docs\TopUpperLimit.png
  // TODO remove?
  // public static final Rotation2d TOP_UPPER_LIMIT = Rotation2d.fromDegrees(0);
  // public static final Rotation2d TOP_LOWER_LIMIT = Rotation2d.fromDegrees(0);
  // public static final Rotation2d BOTTOM_UPPER_LIMIT = Rotation2d.fromDegrees(0);
  // public static final Rotation2d BOTTOM_LOWER_LIMIT = Rotation2d.fromDegrees(0);
  // // the total length of the arm + end effector from the rotational joint, for kinematic use
  // public static final Distance LENGTH_TO_END_EFFECTOR = Meters.of(0);

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
    return Math.abs(inputs.absolutePosition.getRotations() - desiredState.armState.rotations)
        < ARM_TOLERANCE.getRotations();
  }

  // public ArmZone getArmZone() {
  //   return inputs.zone;
  // }

  // public boolean isInIntakeZone() {
  //   return Superstructure.isInIntakeZone(inputs.zone);
  // }

  // /**
  //  * @return {@code false} if the the arm has to go through potential collision zones to get to
  // the
  //  *     desired state from its current state, {@code true} if the current state and desired
  // state
  //  *     are in the same zone.
  //  *     <p>note that if the elevator is high enough, this wont matter, since it will be out of
  // the
  //  *     range of the intake and bellypan
  //  */
  // public boolean willStayInZone(ArmState desiredState) {
  //   return inputs.zone == desiredState.zone;
  // }

  public void setState(ArmState state) {
    desiredState = state;
    io.setDesiredState(desiredState);
  }

  public ArmState getState() {
    return inputs.state;
  }
}
