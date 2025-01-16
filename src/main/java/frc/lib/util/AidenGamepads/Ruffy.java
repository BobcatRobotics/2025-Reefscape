package frc.lib.util.AidenGamepads;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class Ruffy {
  private final CommandJoystick gp;
  public Trigger button;
  public DoubleSupplier xAxis;
  public DoubleSupplier yAxis;
  public DoubleSupplier zAxis;

  /** Axis indices start at 0, button indices start at one -_- */
  public Ruffy(int port) {
    gp = new CommandJoystick(port);
    configureTriggers();
    configureAxes();
  }

  private void configureTriggers() {
    button = gp.button(1);
  }

  private void configureAxes() {
    // y is up/down
    // x is left/right
    // z is twist
    xAxis = () -> -gp.getRawAxis(0);
    yAxis = () -> -gp.getRawAxis(1);
    zAxis = () -> -gp.getRawAxis(2);
  }
}
