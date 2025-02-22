package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class EndEffectorIOTalonFX implements EndEffectorIO {

  public static final double INTOOK_THRESHOLD = Inches.of(2).in(Millimeters);
  public static final double END_EFFEFCTOR_GEAR_RATIO = 11 / 36;

  private TalonFX motor;
  private LaserCan laser;
  private VelocityDutyCycle request;
  private Alert rangingAlert =
      new Alert(
          "Couldnt set end effector ranging mode!! OMG this is really bad!! the robot will EXPLODE!!!!!! fix IMEDIATELY or i'll DIE a GRUESOME and PAINFUL death!",
          AlertType.kWarning);

  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Current> statorCurrent;

  public EndEffectorIOTalonFX(int talonID, int laserCANID) {
    motor = new TalonFX(talonID);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motor.getConfigurator().apply(motorConfig);
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 80;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor.getConfigurator().apply(motorConfig);

    laser = new LaserCan(laserCANID);
    try {
      laser.setRangingMode(RangingMode.SHORT);
      rangingAlert.set(false);
    } catch (ConfigurationFailedException e) {
      e.printStackTrace();
      rangingAlert.set(true);
    }

    request = new VelocityDutyCycle(0);

    velocity = motor.getVelocity();
    statorCurrent = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), velocity, statorCurrent);

    motor.optimizeBusUtilization();
  }

  @Override
  public void setSpeed(double rpm) {
    motor.setControl(request.withVelocity(rpm / 60));
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, statorCurrent);
    inputs.laserCanDistanceMilimeters = laser.getMeasurement().distance_mm;
    inputs.velocity =
        RotationsPerSecond.of(motor.getVelocity().getValueAsDouble() * END_EFFEFCTOR_GEAR_RATIO);
    inputs.currentDraw = motor.getStatorCurrent().getValueAsDouble();
    inputs.hasPiece = inputs.laserCanDistanceMilimeters < INTOOK_THRESHOLD;
    inputs.motorConnected = motor.isConnected();
  }
}
