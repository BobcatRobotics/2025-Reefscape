package frc.robot.AidensGamepads;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class EightBitDo extends CommandJoystick {
  public Trigger a;
  public Trigger b;
  public Trigger x;
  public Trigger y;
  public Trigger lb;
  public Trigger rb;
  public Trigger select;
  public Trigger start;
  public Trigger leftPaddle;
  public Trigger rightPaddle;
  public DoubleSupplier leftXAxis;
  public DoubleSupplier leftYAxis;
  public DoubleSupplier rightXAxis;
  public DoubleSupplier rightYAxis;
  public Trigger povUp;
  public Trigger povDown;
  public Trigger povLeft;
  public Trigger povRight;
  public Trigger povCenter;
  public Trigger povDownLeft;
  public Trigger povDownRight;
  public Trigger povUpLeft;
  public Trigger povUpRight;

  /**
   * 8bitdo controller
   *
   * <p>Buttons 1 -b 2 -a 3- y 4 -x 5 -lb 6 - rb 7 - select 8 - start 9 - bl 10 - br
   *
   * <p>Axes 0 - 1 - 2 - LT 3 - RT 4 - 5 -
   *
   * <p>Axis indices start at 0, button indices start at one -_-
   */
  public EightBitDo(int port) {

    super(port);
    a = super.button(2);
    b = super.button(1);
    x = super.button(4);
    y = super.button(3);
    lb = super.button(5);
    rb = super.button(6);
    select = super.button(7);
    start = super.button(8);
    leftPaddle = super.button(9);
    rightPaddle = super.button(10);
    povUp = super.povUp();
    povDown = super.povDown();
    povLeft = super.povLeft();
    povRight = super.povRight();
    povCenter = super.povCenter();
    povDownLeft = super.povDownLeft();
    povDownRight = super.povDownRight();
    povUpLeft = super.povUpLeft();
    povUpRight = super.povUpRight();

    leftXAxis = () -> super.getRawAxis(0);
    leftYAxis = () -> -super.getRawAxis(1);
    rightXAxis = () -> super.getRawAxis(4);
    rightYAxis = () -> -super.getRawAxis(3);
  }
}
