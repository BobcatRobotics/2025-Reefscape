package frc.robot.AidensGamepads;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBoard extends CommandJoystick {
  public Trigger l1;
  public Trigger l2;
  public Trigger l3;
  public Trigger l4;
  public Trigger net;

  public ButtonBoard(int port) {
    super(port);
    l1 = super.button(1); // TODO double check these
    l2 = super.button(2);
    l3 = super.button(3);
    l4 = super.button(4);
    net = super.button(5);
  }
}
