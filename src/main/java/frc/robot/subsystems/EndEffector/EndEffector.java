// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EndEffector.EndEffectorIO.EndEffectorIOInputs;

public class EndEffector extends SubsystemBase {
  private EndEffectorIOInputs inputs = new EndEffectorIOInputsAutoLogged();
  private EndEffectorIO io;
  
  /** Creates a new EndEffector. */
  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
  
  public Distance getDistanceToPiece(){
    return Meters.of(inputs.laserCanDistanceMeters);
  }
  public void setSpeed(double rpm){
    io.setSpeed(rpm);
  }
}
