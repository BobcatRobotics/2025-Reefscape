package frc.robot.subsystems.CoralIntake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.Constants;
import org.littletonrobotics.junction.Logger;

public class CoralIntakeIOSim implements CoralIntakeIO {
  private static final double autoStartAngle = Units.degreesToRadians(90.0);
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(10),
          1,
          0.2,
          0.6096,
          -1000,
          1000,
          false,
          Units.degreesToRadians(0.0));

  private final PIDController controller;
  private double appliedVoltage = 0.0;
  private double positionOffset = 0.0;
  private boolean controllerNeedsReset = false;
  private boolean wasNotAuto = true;
  private IntakeState desiredState = IntakeState.UNKNOWN;
  private boolean closedLoop = true;

  private Current rollerVelocity = Amps.of(0);

  public CoralIntakeIOSim() {
    controller = new PIDController(1.25, 0.0, 0.25);
    sim.setState(0.0, 0.0);
    controller.setTolerance(Math.toRadians(2));
    setPosition(0.0);
  }

  @Override
  public void updateInputs(CoralIntakeIOInputs inputs) {
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

    //     public boolean hasPiece = false;
    // public Distance LaserCANDistance = Millimeters.of(0);
    // public boolean pivotMotorConnected = false;
    // public boolean rollerMotorConnected = false;
    // public double intakeVelocityRPM = 0;
    // public double desiredRollerVelocityRPM = 0;
    // public double desiredCarwashVelocityRPM = 0;
    // public double laserCanDistanceMilimeters = 0;
    // public double positionRotations = 0;
    // public IntakeState state = IntakeState.RETRACT;

    inputs.positionRotations = sim.getAngleRads() / (2 * Math.PI);
    inputs.state = desiredState;
    inputs.rollerMotorConnected = true;
    inputs.pivotMotorConnected = true;
    inputs.desiredRollerVelocityRPM = rollerVelocity.baseUnitMagnitude();

    Logger.recordOutput("desiredAngleRads", desiredState.simAngle.in(Radians));
    runVolts(controller.calculate(sim.getAngleRads(), desiredState.simAngle.in(Radians)));
  }

  @Override
  public void setState(IntakeState state) {
    desiredState = state;
    setPos(state.simAngle.in(Radians));
  }

  private void setPos(double rads) {
    if (!closedLoop) {
      controllerNeedsReset = true;
      closedLoop = true;
    }
    // double radians = flipped ? 0.5 - state.rotations : state.rotations;

    if (controllerNeedsReset) {
      controller.reset();
      controllerNeedsReset = false;
    }
    controller.setSetpoint(rads);
  }

  private void runVolts(double volts) {
    closedLoop = false;
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  private void setPosition(double position) {
    closedLoop = true;
    positionOffset = position - sim.getAngleRads();
  }

  @Override
  public void deploy(Angle trim) {
    setPos(-8.5);
  }

  @Override
  public void retract() {
    setState(IntakeState.RETRACT);
  }

  @Override
  public void setSpeed(Current velocity) {
    velocity = rollerVelocity;
  }
}
