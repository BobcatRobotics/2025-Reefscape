// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AidenGamepads;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class LogitechJoystick extends CommandJoystick {

  public Trigger trigger;
  public Trigger thumb;
  public Trigger topRight;
  public Trigger topLeft;
  public Trigger bottomRight;
  public Trigger bottomLeft;
  public DoubleSupplier xAxis;
  public DoubleSupplier yAxis;
  public DoubleSupplier zAxis;
  public DoubleSupplier throttle;

  public Trigger bottom7;
  public Trigger bottom8;
  public Trigger bottom9;
  public Trigger bottom10;
  public Trigger bottom11; // button is broken on controller?
  public Trigger bottom12;

  // TODO add bottom buttons and dpad
  public LogitechJoystick(int port) {
    super(port);
    trigger = super.button(1);
    thumb = super.button(2);
    topRight = super.button(6);
    topLeft = super.button(5);
    bottomRight = super.button(4);
    bottomLeft = super.button(3);
    xAxis = () -> super.getRawAxis(0);
    yAxis = () -> super.getRawAxis(1);
    zAxis = () -> super.getRawAxis(2);
    throttle = () -> -super.getRawAxis(3);

    bottom7 = super.button(7);
    bottom8 = super.button(8);
    bottom9 = super.button(9);
    bottom10 = super.button(10);
    bottom11 = super.button(11);
    bottom12 = super.button(12);
  }
}
