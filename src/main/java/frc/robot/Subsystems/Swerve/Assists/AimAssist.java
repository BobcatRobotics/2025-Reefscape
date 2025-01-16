package frc.robot.Subsystems.Swerve.Assists;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AimAssist {
  private Supplier<Translation2d> transSupplier;
  private Supplier<Rotation2d> rotSupplier;
  private BooleanSupplier activateTrans;
  private BooleanSupplier activateRot;
  public PIDController pidX;
  public PIDController pidY;
  public PIDController pidTheta;


  public AimAssist(
      Supplier<Translation2d> transSupplier,
      Supplier<Rotation2d> rotSupplier,
      BooleanSupplier activateTrans,
      BooleanSupplier activateRot,
      double kp,
      double ki,
      double kd,
      double kptheta,
      double kitheta,
      double kdtheta) {
    this(transSupplier, rotSupplier, activateTrans, activateRot, kp, ki, kd, kp, ki, kd, kptheta, kitheta, kdtheta);
  }

  public AimAssist(
    Supplier<Translation2d> transSupplier,
    Supplier<Rotation2d> rotSupplier,
    BooleanSupplier activateTrans,
      BooleanSupplier activateRot,
      double kpx,
      double kix,
      double kdx,
      double kpy,
      double kiy,
      double kdy,
      double kptheta,
      double kitheta,
      double kdtheta) {

    this.transSupplier = transSupplier;
    this.rotSupplier = rotSupplier;
    this.activateTrans = activateTrans;
    this.activateRot = activateRot;

    pidX = new PIDController(kpx, kix, kdx);
    pidY = new PIDController(kpy, kiy, kdy);
    pidTheta = new PIDController(kptheta, kitheta, kdtheta);

    pidTheta.enableContinuousInput(0, 2*Math.PI);
  }

  /**
   * 
   * @param pose
   * @return euclidian distance to target, 0 if pose is null not active
   */
  public double distanceToTarget(Translation2d pose) {
    return pose.getDistance(transSupplier.get());
  }
    
  

  public Rotation2d distanceToTarget(Rotation2d rot){
    return rot.minus(rotSupplier.get());
  }

  public double outputX(Translation2d pose) {
    return pidX.calculate(transSupplier.get().getX(), pose.getX());
  }

  public double outputY(Translation2d pose) {
    return pidX.calculate(transSupplier.get().getY(), pose.getY());
  }

  public double outputTheta(Rotation2d currentYaw) {
    return pidTheta.calculate(rotSupplier.get().getRadians(), currentYaw.getRadians());
  }


  public boolean rotActive() {
    return activateRot.getAsBoolean();
  }
  public boolean transActive(){
    return activateTrans.getAsBoolean();
  }
}
