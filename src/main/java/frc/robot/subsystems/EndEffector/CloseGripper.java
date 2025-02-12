// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CloseGripper extends Command {
  EndEffector effector;
  private double motorSpeed = 1.0;
  private double holdSpeed = 0.1;
  /** Creates a new CloseGripper. */
  public CloseGripper(EndEffector effector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.effector = effector;
    addRequirements(this.effector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    effector.run(motorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    effector.run(holdSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
