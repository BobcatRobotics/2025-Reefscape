// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmIO.ArmIOInputs;
import frc.robot.subsystems.StateMachine.StateObserver;
import frc.robot.subsystems.StateMachine.SuperstructureState;
import org.littletonrobotics.junction.AutoLogOutput;

public class Arm extends SubsystemBase {
  // see Assets\Docs\TopUpperLimit.png
  public static final Rotation2d TOP_UPPER_LIMIT = Rotation2d.fromDegrees(0);
  public static final Rotation2d TOP_LOWER_LIMIT = Rotation2d.fromDegrees(0);
  public static final Rotation2d BOTTOM_UPPER_LIMIT = Rotation2d.fromDegrees(0);
  public static final Rotation2d BOTTOM_LOWER_LIMIT = Rotation2d.fromDegrees(0);
  // the total length of the arm + end effector from the rotational joint, for kinematic use
  public static final Distance LENGTH_TO_END_EFFECTOR = Meters.of(0);

  private Alert motorDisconnected = new Alert("Arm motor disconnected!", AlertType.kWarning);
  private Alert encoderDisconnected = new Alert("Arm motor disconnected!", AlertType.kWarning);

  ArmIO io;
  ArmIOInputs inputs = new ArmIOInputsAutoLogged();
  private ArmState desiredState = ArmState.NO_OP;

  /** Creates a new Arm. */
  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    StateObserver.getInstance().updateArm(inputs.state, inputs.position);
    io.setDesiredState(desiredState);
    motorDisconnected.set(!inputs.motorConnected);
    encoderDisconnected.set(!inputs.encoderConnected);
  }

  public static ArmZone getArmZone(ArmState state) {
    return getArmZone(Rotation2d.fromDegrees(state.degrees));
  }

  public static ArmZone getArmZone(SuperstructureState goal) {
    return getArmZone(goal.armState);
  }
  /** see Assets\Docs\TopUpperLimit.png */
  public static ArmZone getArmZone(Rotation2d position) {
    double deg = position.getDegrees();
    if (deg >= TOP_LOWER_LIMIT.getDegrees() && deg <= TOP_UPPER_LIMIT.getDegrees()) {
      return ArmZone.TOP_ZONE;
    } else if (deg > TOP_UPPER_LIMIT.getDegrees() && deg < BOTTOM_LOWER_LIMIT.getDegrees()) {
      return ArmZone.CORAL_INTAKE;
    } else if (deg > BOTTOM_LOWER_LIMIT.getDegrees() && deg < BOTTOM_UPPER_LIMIT.getDegrees()) {
      return ArmZone.BOTTOM_ZONE;
    } else {
      return ArmZone.ALGAE_INTAKE;
    }
  }

  public ArmZone getArmZone() {
    return getArmZone(inputs.position);
  }

  public boolean isInIntakeZone() {
    return StateObserver.isInIntakeZone(getArmZone());
  }

  /**
   * @return {@code false} if the the arm has to go through potential collision zones to get to the
   *     desired state from its current state, {@code true} if the current state and desired state
   *     are in the same zone.
   *     <p>note that if the elevator is high enough, this wont matter, since it will be out of the
   *     range of the intake and bellypan
   */
  public boolean willStayInZone(ArmState desiredState) {
    return getArmZone() == desiredState.zone;
  }

  public void setState(ArmState state) {
    desiredState = state;
  }

  public ArmState getState() {
    return inputs.state;
  }
}
