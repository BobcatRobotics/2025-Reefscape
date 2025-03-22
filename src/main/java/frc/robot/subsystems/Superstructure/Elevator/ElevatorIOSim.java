package frc.robot.subsystems.Superstructure.Elevator;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Constants;

public class ElevatorIOSim implements ElevatorIO {
  public static final double carriageMassKg = Units.lbsToKilograms(6.0);
  public static final double stagesMassKg = Units.lbsToKilograms(8.0);
  public static final DCMotor gearbox =
      DCMotor.getKrakenX60Foc(1000).withReduction(ElevatorIOTalonFX.GEAR_RATIO);

  public static final Matrix<N2, N2> A =
      MatBuilder.fill(
          Nat.N2(),
          Nat.N2(),
          0,
          1,
          0,
          -gearbox.KtNMPerAmp
              / (gearbox.rOhms
                  * Math.pow(Elevator.DRUM_RADIUS.baseUnitMagnitude(), 2)
                  * (carriageMassKg + stagesMassKg)
                  * gearbox.KvRadPerSecPerVolt));
  public static final Vector<N2> B =
      VecBuilder.fill(
          0.0,
          gearbox.KtNMPerAmp
              / (Elevator.DRUM_RADIUS.baseUnitMagnitude() * (carriageMassKg + stagesMassKg)));

  // State given by elevator carriage position and velocity
  // Input given by torque current to motor
  private Vector<N2> simState;
  private double inputTorqueCurrent = 0.0;
  private double appliedVolts = 0.0;

  private final PIDController controller = new PIDController(20000.0, 0.0, 0.0);
  private boolean closedLoop = false;
  private double feedforward = 0.0;

  public ElevatorIOSim() {
    simState = VecBuilder.fill(0.0, 0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (!closedLoop) {
      controller.reset();
      update(Constants.loopPeriodSecs);
    } else {
      // Run control at 1khz
      for (int i = 0; i < Constants.loopPeriodSecs / (1.0 / 1000.0); i++) {
        setInputTorqueCurrent(
            controller.calculate(simState.get(0) / Elevator.DRUM_RADIUS.baseUnitMagnitude())
                + feedforward);
        update(1.0 / 1000.0);
      }
    }

    // boolean motorConnected,
    // boolean followerConnected,
    // double positionRad,
    // double velocityRadPerSec,
    // double appliedVolts,
    // double torqueCurrentAmps,
    // double supplyCurrentAmps,
    // double tempCelsius,
    // double followerAppliedVolts,
    // double followerTorqueCurrentAmps,
    // double followerSupplyCurrentAmps,
    // double followerTempCelsius) {}

    inputs.motorConnected = true;

    inputs.rotPosition =
        Rotation2d.fromRotations(simState.get(0) / Elevator.DRUM_RADIUS.baseUnitMagnitude());
    inputs.positionPercent = inputs.rotPosition.getRadians() / Elevator.MAX_ROTATIONS.getRadians();
    inputs.positionRotations = inputs.rotPosition.getRotations();
    inputs.velocityRotPerSec = simState.get(1) / Elevator.DRUM_RADIUS.baseUnitMagnitude();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = Math.copySign(inputTorqueCurrent, appliedVolts);
    inputs.heightMeters = inputs.rotPosition.getRotations() * Elevator.METERS_PER_ROTATION;
    inputs.aligned = controller.atSetpoint();
    inputs.overriden = !closedLoop;
    inputs.distanceToAlignment = controller.getPositionError();
    inputs.closedLoopReference = controller.getSetpoint();
    inputs.closedLoopReferenceSlope = 0.0;
    inputs.supplyCurrentAmps = Math.copySign(inputTorqueCurrent, appliedVolts);
  }

  @Override
  public void manualOverride(double output) {
    closedLoop = false;
    setInputTorqueCurrent(output * gearbox.stallCurrentAmps);
  }

  @Override
  public void setDesiredState(ElevatorState state) {
    closedLoop = true;
    controller.setSetpoint(state.pos.getRotations());
  }

  private void setInputTorqueCurrent(double torqueCurrent) {
    inputTorqueCurrent = torqueCurrent;
    appliedVolts =
        gearbox.getVoltage(
            gearbox.getTorque(inputTorqueCurrent),
            simState.get(1, 0) / Elevator.DRUM_RADIUS.baseUnitMagnitude());
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
  }

  private void setInputVoltage(double voltage) {
    setInputTorqueCurrent(
        gearbox.getCurrent(simState.get(1) / Elevator.DRUM_RADIUS.baseUnitMagnitude(), voltage));
  }

  private void update(double dt) {
    inputTorqueCurrent =
        MathUtil.clamp(inputTorqueCurrent, -gearbox.stallCurrentAmps, gearbox.stallCurrentAmps);
    Matrix<N2, N1> updatedState =
        NumericalIntegration.rkdp(
            (Matrix<N2, N1> x, Matrix<N1, N1> u) ->
                A.times(x).plus(B.times(u)).plus(VecBuilder.fill(0.0, -9.80665)),
            simState,
            MatBuilder.fill(Nat.N1(), Nat.N1(), inputTorqueCurrent),
            dt);
    // Apply limits
    simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
    if (simState.get(0) <= 0.0) {
      simState.set(1, 0, 0.0);
      simState.set(0, 0, 0.0);
    }
    if (simState.get(0) >= Elevator.MAX_HEIGHT.baseUnitMagnitude()) {
      simState.set(1, 0, 0.0);
      simState.set(0, 0, Elevator.MAX_HEIGHT.baseUnitMagnitude());
    }
  }
}
