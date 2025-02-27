package frc.robot.subsystems.CoralIntake;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class CoralIntakeIOTalonFX implements CoralIntakeIO {

  public static double MAX_ROTATIONS = 14; // TODO slightly more than this

  private TalonFX roller;
  private TalonFX pivot;
  private LaserCan laser;
  private PositionVoltage positionRequest = new PositionVoltage(0);
  private VelocityVoltage rollerRequest = new VelocityVoltage(0);
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private Alert rangingAlert =
      new Alert(
          "Couldnt set intake ranging mode!! OMG this is really bad!! the robot will EXPLODE!!!!!! fix IMEDIATELY or i'll DIE a GRUESOME and PAINFUL death!",
          AlertType.kWarning);

  private StatusSignal<AngularVelocity> rollerVelocity;
  private StatusSignal<Angle> position;

  private AngularVelocity desiredRollerVelocity = RotationsPerSecond.of(0);

  private IntakeState currState = IntakeState.UNKNOWN;

  public CoralIntakeIOTalonFX(int rollerMotorID, int pivotMotorID, int laserCANID) {

    // laser = new LaserCan(laserCANID);

    // try {
    // laser.setRangingMode(RangingMode.SHORT);
    // rangingAlert.set(false);
    // } catch (ConfigurationFailedException e) {
    // e.printStackTrace();
    // rangingAlert.set(true);
    // }

    roller = new TalonFX(rollerMotorID);
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    roller.getConfigurator().apply(rollerConfig);
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 80;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO check
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.Slot0.kP = 0.3; // // TODO tune
    roller.getConfigurator().apply(rollerConfig);

    pivot = new TalonFX(pivotMotorID);
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivot.getConfigurator().apply(pivotConfig);
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 60;
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // TODO check
    pivotConfig.Slot0.kP = 1; // TODO tune
    pivot.getConfigurator().apply(pivotConfig);
    pivot.setPosition(0);

    rollerVelocity = roller.getVelocity();
    position = pivot.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), rollerVelocity, position);

    pivot.optimizeBusUtilization();
    roller.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(CoralIntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(rollerVelocity, position);
    inputs.intakeVelocityRPM = rollerVelocity.getValue().in(Rotations.per(Minute));
    inputs.position = position.getValue();
    inputs.pivotMotorConnected = pivot.isConnected();
    inputs.rollerMotorConnected = roller.isConnected();
    inputs.state = currState;
    inputs.desiredRollerVelocityRPM = desiredRollerVelocity.in(Rotations.per(Minute));
  }

  @Override
  public void setSpeed(double output) {
    roller.setControl(dutyCycleOut.withOutput(output));
    // roller.setControl(rollerRequest.withVelocity(velocity));
    // desiredRollerVelocity = velocity;
  }

  @Override
  public void setAngle(Angle angle) {
    pivot.setControl(positionRequest.withPosition((angle)));
    currState = IntakeState.UNKNOWN;
  }

  @Override
  public void setState(IntakeState state) {
    pivot.setControl(positionRequest.withPosition((state.angle)));
    currState = state;
  }

  @Override
  public void retract() {
    pivot.setControl(positionRequest.withPosition((IntakeState.RETRACT.angle)));
  }

  @Override
  public void deploy() {
    pivot.setControl(positionRequest.withPosition((IntakeState.DEPLOY.angle)));
  }

  @Override
  public void stop() {
    roller.stopMotor();
  }
}

// fun fact: A group of Anands is called a department
