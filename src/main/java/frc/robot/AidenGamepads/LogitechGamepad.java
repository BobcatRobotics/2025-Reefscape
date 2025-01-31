package frc.robot.AidenGamepads;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class LogitechGamepad extends CommandJoystick {
  public Trigger a;
  public Trigger b;
  public Trigger x;
  public Trigger y;
  public Trigger lb;
  public Trigger rb;
  public Trigger back;
  public Trigger start;
  public Trigger leftStick;
  public Trigger rightStick;
  public Trigger povUp;
  public Trigger povDown;
  public Trigger povLeft;
  public Trigger povRight;
  public Trigger povCenter;
  public Trigger povDownLeft;
  public Trigger povDownRight;
  public Trigger povUpLeft;
  public Trigger povUpRight;

  public DoubleSupplier leftXAxis;
  public DoubleSupplier leftYAxis;
  public DoubleSupplier rightXAxis;
  public DoubleSupplier rightYAxis;

  // TODO test
  /**
   * Logitech controller
   *
   * <p>Buttons 1 - x 2 - a 3- b 4 - y 5 - lb 6 - rb 7 - lt 8 - rt 9 - back 10 - start 11 - lstick
   * 12 - rstick
   *
   * <p>Axes 0 - lstick x 1 - lstick y 2 - rstick x 3 - rstick y
   *
   * <p>Axis indices start at 0, button indices start at one -_-
   */
  public LogitechGamepad(int port) {
    super(port);
    configureTriggers();
    configureAxes();
  }

  private void configureTriggers() {
    a = super.button(1);
    b = super.button(2);
    x = super.button(3);
    y = super.button(4);
    lb = super.button(5); // TODO check
    rb = super.button(6);
    back = super.button(9);
    start = super.button(10);
    leftStick = super.button(11);
    rightStick = super.button(12);
    povUp = super.povUp();
    povDown = super.povDown();
    povLeft = super.povLeft();
    povRight = super.povRight();
    povCenter = super.povCenter();
    povDownLeft = super.povDownLeft();
    povDownRight = super.povDownRight();
    povUpLeft = super.povUpLeft();
    povUpRight = super.povUpRight();
  }

  private void configureAxes() {
    // y is up/down
    // x is left/right
    leftXAxis = () -> super.getRawAxis(0);
    leftYAxis = () -> -super.getRawAxis(1);
    rightXAxis = () -> super.getRawAxis(4);
    rightYAxis = () -> -super.getRawAxis(5);
  }
}
