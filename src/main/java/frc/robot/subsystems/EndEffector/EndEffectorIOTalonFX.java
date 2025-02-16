package frc.robot.subsystems.EndEffector;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EndEffectorIOTalonFX implements EndEffectorIO {
  public static final double END_EFFEFCTOR_GEAR_RATIO = 11 / 36;

  private TalonFX motor;
  private LaserCan laser;
  private VelocityDutyCycle request;
  private Alert rangingAlert = new Alert("Couldnt set end effector ranging mode!! OMG this is really bad!! the robot will EXPLODE!!!!!! fix IMEDIATELY", AlertType.kWarning);

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
    motor.setControl(request.withVelocity(rpm/60)); 
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, statorCurrent);
    inputs.laserCanDistanceMeters = laser.getMeasurement().distance_mm / 1000;
    inputs.rpm = motor.getVelocity().getValueAsDouble() * 60 * END_EFFEFCTOR_GEAR_RATIO;
    inputs.currentDraw = motor.getStatorCurrent().getValueAsDouble();
  }
}
