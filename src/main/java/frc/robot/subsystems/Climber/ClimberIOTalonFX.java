package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public class ClimberIOTalonFX implements ClimberIO {
  private TalonFX motor;
  private PositionVoltage positionRequest = new PositionVoltage(0);
  private DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  private StatusSignal<Angle> position;

  public ClimberIOTalonFX(int talonID) {
    motor = new TalonFX(talonID);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.Slot0.kP = 1; // TODO tuner
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 60; // TODO tune
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 66 + 60;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -145 + 60;
    motor.getConfigurator().apply(motorConfig);
    motor.setPosition(0);

    position = motor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), position);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.connected = motor.isConnected();
    inputs.positionRotations = position.getValueAsDouble();
  }

  @Override
  public void setDutyCycle(double output) {
    output = MathUtil.clamp(output, -1, 1);
    motor.setControl(dutyCycleRequest.withOutput(output));
  }

  @Override
  public void setPosition(Rotation2d position) {}
}
