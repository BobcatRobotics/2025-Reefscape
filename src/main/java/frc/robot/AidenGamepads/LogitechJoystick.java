// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AidenGamepads;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class LogitechJoystick extends CommandJoystick {
    public Trigger topRight;
    public Trigger topLeft;
    public Trigger bottomRight;
    public Trigger bottomLeft;
    public DoubleSupplier xAxis;
    public DoubleSupplier yAxis;
    public DoubleSupplier zAxis;
    //TODO add bottom buttons and dpad
    public LogitechJoystick(int port) {
        super(port);
        //topRight = super.button(0);
        
    }

}
