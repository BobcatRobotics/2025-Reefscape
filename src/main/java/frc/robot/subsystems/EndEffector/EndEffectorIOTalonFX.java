package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class EndEffectorIOTalonFX implements EndEffectorIO {

  public static final double INTOOK_THRESHOLD = Inches.of(1).in(Millimeters);
  public static final double END_EFFEFCTOR_GEAR_RATIO = 11 / 36;

  private TalonFX motor;
  private LaserCan laser;
  private VelocityVoltage request = new VelocityVoltage(0);
  private Alert rangingAlert =
      new Alert(
          "Couldnt set end effector ranging mode!! OMG this is really bad!! the robot will EXPLODE!!!!!! fix IMEDIATELY or i'll DIE a GRUESOME and PAINFUL death!",
          AlertType.kWarning);

  private double desiredSpeed = 0;

  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Current> statorCurrent;

  public EndEffectorIOTalonFX(int talonID, int laserCANID) {
    motor = new TalonFX(talonID);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motor.getConfigurator().apply(motorConfig);
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 80;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.Slot0.kP = 0.36;
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

    velocity = motor.getVelocity();
    statorCurrent = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), velocity, statorCurrent);

    motor.optimizeBusUtilization();
  }

  @Override
  public void setSpeed(double rps) {
    motor.setControl(request.withVelocity(rps));
    desiredSpeed = rps;
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, statorCurrent);
    inputs.laserCanDistanceMilimeters =
        laser.getMeasurement() == null ? -1 : laser.getMeasurement().distance_mm;
    inputs.velocityRPS = motor.getVelocity().getValueAsDouble();
    inputs.currentDraw = motor.getStatorCurrent().getValueAsDouble();
    inputs.hasPiece =
        (inputs.laserCanDistanceMilimeters < INTOOK_THRESHOLD)
            && inputs.laserCanDistanceMilimeters != -1;
    inputs.motorConnected = motor.isConnected();
    inputs.desiredSpeedRPS = desiredSpeed;
  }
}
