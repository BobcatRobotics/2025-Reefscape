package frc.robot.AidensGamepads;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class Ruffy extends CommandJoystick {

  public Trigger button;
  public DoubleSupplier xAxis;
  public DoubleSupplier yAxis;
  public DoubleSupplier zAxis;

  /** Axis indices start at 0, button indices start at one -_- */
  public Ruffy(int port) {
    super(port);
    configureTriggers();
    configureAxes();
  }

  private void configureTriggers() {
    button = super.button(1);
  }

  private void configureAxes() {
    // y is up/down
    // x is left/right
    // z is twist
    xAxis = () -> super.getRawAxis(0);
    yAxis = () -> -super.getRawAxis(1);
    zAxis = () -> super.getRawAxis(2);
  }
}
