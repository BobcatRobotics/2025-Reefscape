package frc.lib.util.AidenGamepads;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class Logitech {
  private final CommandJoystick gp;
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
  public Logitech(int port) {
    gp = new CommandJoystick(port);
    configureTriggers();
    configureAxes();
  }

  private void configureTriggers() {
    a = gp.button(2);
    b = gp.button(3);
    x = gp.button(1);
    y = gp.button(4);
    lb = gp.button(5);
    rb = gp.button(6);
    back = gp.button(9);
    start = gp.button(10);
    leftStick = gp.button(11);
    rightStick = gp.button(12);
    povUp = gp.povUp();
    povDown = gp.povDown();
    povLeft = gp.povLeft();
    povRight = gp.povRight();
    povCenter = gp.povCenter();
    povDownLeft = gp.povDownLeft();
    povDownRight = gp.povDownRight();
    povUpLeft = gp.povUpLeft();
    povUpRight = gp.povUpRight();
  }

  private void configureAxes() {
    // y is up/down
    // x is left/right
    leftXAxis = () -> -gp.getRawAxis(0);
    leftYAxis = () -> -gp.getRawAxis(1);
    rightXAxis = () -> -gp.getRawAxis(2);
    rightYAxis = () -> -gp.getRawAxis(3);
  }
}
