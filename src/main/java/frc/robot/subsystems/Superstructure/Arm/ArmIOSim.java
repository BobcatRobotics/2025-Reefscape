package frc.robot.subsystems.Superstructure.Arm;

import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.Constants;

public class ArmIOSim implements ArmIO {
  private static final double autoStartAngle = Units.degreesToRadians(90.0);
  private static final double armLength = Units.inchesToMeters(20.145958); // from cad
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(2),
          83.06, // TODO this is probably wrong?
          1.06328,
          0.6096,
          -1000,
          1000,
          false,
          Units.degreesToRadians(0.0));

  private final PIDController controller;
  private double appliedVoltage = 0.0;
  private double positionOffset = 0.0;

  private boolean controllerNeedsReset = false;
  private boolean closedLoop = true;
  private boolean wasNotAuto = true;
  private boolean isFlipped = false;
  private boolean isInUnkownState = true;
  private ArmState desiredState = ArmState.UNKOWN;
  private double desiredRads = 0.0;

  public ArmIOSim() {
    controller = new PIDController(10.0, 0.0, 0.0);
    sim.setState(0.0, 0.0);
    controller.setTolerance(Math.toRadians(2));
    setPosition(0.0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      controllerNeedsReset = true;
    }

    // Reset at start of auto
    if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
      sim.setState(autoStartAngle, 0.0);
      wasNotAuto = false;
    }
    wasNotAuto = !DriverStation.isAutonomousEnabled();

    sim.update(Constants.loopPeriodSecs);

    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps();
    inputs.isOverridden = !closedLoop;
    inputs.encoderConnected = true;
    inputs.motorConnected = true;
    inputs.velocityRotPerSec = sim.getVelocityRadPerSec() / (2 * Math.PI);
    inputs.absolutePosition = Rotation2d.fromRadians(sim.getAngleRads());
    inputs.positionDegrees = inputs.absolutePosition.getDegrees();
    inputs.desiredPositionRotation = controller.getSetpoint() / (2 * Math.PI);
    inputs.distanceToAlignment =
        Math.abs(inputs.absolutePosition.getRotations() - inputs.desiredPositionRotation) * 360;
    inputs.flipped = isFlipped;
    inputs.controlMode = ControlModeValue.Reserved;
    inputs.aligned = controller.atSetpoint();
    inputs.appliedVolts = appliedVoltage;
    inputs.state = desiredState;

    runVolts(controller.calculate(sim.getAngleRads(), desiredRads));
  }

  @Override
  public void setDesiredState(ArmState state, boolean flipped, boolean hasPiece) {
    desiredState = state;
    if (state == ArmState.NO_OP || state == ArmState.UNKOWN) {
      isInUnkownState = true;
      return;
    } else {
      isInUnkownState = false;
    }

    double radians = flipped ? 0.5 - state.rotations : state.rotations;
    radians *= 2 * Math.PI;
    desiredRads = radians;

    isFlipped = flipped;
    if (!closedLoop) {
      controllerNeedsReset = true;
      closedLoop = true;
    }
    if (controllerNeedsReset) {
      controller.reset();
      controllerNeedsReset = false;
    }
    controller.setSetpoint(radians + positionOffset);
  }

  private void runVolts(double volts) {
    closedLoop = false;
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void manualOverride(double percent) {
    runVolts(percent * 12.0);
  }

  private void setPosition(double position) {
    closedLoop = true;
    positionOffset = position - sim.getAngleRads();
  }
}
