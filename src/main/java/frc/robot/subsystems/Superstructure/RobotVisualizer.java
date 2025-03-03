package frc.robot.subsystems.Superstructure;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {
  // Singleton instance
  private static RobotVisualizer instance;

  private double elevatorHeightMeters = 0;
  private double armAngleRadians = 0;
  private double intakeAngleRadians = 0;
  private final Rotation2d minArmAngle = Rotation2d.kCW_90deg;

  private double desiredElevatorHeightMeters = 0;
  private double desiredArmAngleRadians = 0;
  private double desiredIntakeAngleRadians = 0;

  private RobotVisualizer() {}

  // Singleton accessor
  public static RobotVisualizer getInstance() {
    if (instance == null) {
      instance = new RobotVisualizer();
    }
    return instance;
  }

  public void setElevatorHeight(Distance height) {
    // Consider adding bounds checking if necessary.
    // prevent cad from going through the floor when the elevator isnt initialized
    elevatorHeightMeters = height.in(Meters) == -1 ? 0 : height.in(Meters);
    updateReal();
  }

  /** 0 deg is horizontal pointing right */
  public void setArmRotation(Rotation2d angle) {
    armAngleRadians = angle.getRadians();
    updateReal();
  }

  /** 0 deg is all the way back/retracted */
  public void setIntakeRotation(Rotation2d angle) {
    intakeAngleRadians = angle.getRadians();
    updateReal();
  }

  public void setDesiredSuperstructureState(SuperstructureState state) {
    desiredArmAngleRadians = state.armState.radians;
    desiredElevatorHeightMeters = state.elevatorState.heightMeters;
    updateDesired();
  }

  public void setDesiredElevatorHeight(Distance height) {
    desiredElevatorHeightMeters = height.in(Meters);
    updateDesired();
  }

  /** 0 deg is horizontal pointing right */
  public void setDesiredArmRotation(Rotation2d angle) {
    desiredArmAngleRadians = angle.getRadians();
    updateDesired();
  }

  /** 0 deg is all the way back/retracted */
  public void setDesiredIntakeRotation(Rotation2d angle) {
    desiredIntakeAngleRadians = angle.getRadians();
    updateDesired();
  }

  private void updateReal() {
    // the distance between each stage of the elevator
    double stageHeight = elevatorHeightMeters * oscilator() / 3; // elevatorHeightMeters / 3;
    double intakeAngle = minArmAngle.getRadians() - intakeAngleRadians;

    Logger.recordOutput("Visualization/Debug/ZeroedRobotPose", new Pose2d());
    Logger.recordOutput("Visualization/Debug/ZeroedMechanismPose", new Pose3d[] {new Pose3d()});
    Logger.recordOutput(
        "Visualization/Mechanisms",
        new Pose3d[] {
          new Pose3d(0, -0.192, 0.14 + stageHeight, new Rotation3d()), // stage 1
          new Pose3d(0, -0.192, 0.16 + stageHeight * 2, new Rotation3d()), // stage 2
          new Pose3d(0, -0.192, 0.18 + stageHeight * 3, new Rotation3d()), // carrige
          new Pose3d(
              0, -0.21, 0.313 + stageHeight * 3, new Rotation3d(0, armAngleRadians, 0)), // arm
          new Pose3d(-0.282, 0.01, 0.305, new Rotation3d(0, intakeAngle, 0)) // intake
        });
  }

  private void updateDesired() {
    double stageHeight = desiredElevatorHeightMeters / 3;
    double intakeAngle = minArmAngle.getRadians() - desiredArmAngleRadians;
    Logger.recordOutput(
        "Visualization/DesiredMechanismPositions",
        new Pose3d[] {
          new Pose3d(0, -0.192, 0.14 + stageHeight, new Rotation3d()), // stage 1
          new Pose3d(0, -0.192, 0.16 + stageHeight * 2, new Rotation3d()), // stage 2
          new Pose3d(0, -0.192, 0.18 + stageHeight * 3, new Rotation3d()), // carrige
          new Pose3d(
              0, -0.21, 0.313 + stageHeight * 3, new Rotation3d(0, armAngleRadians, 0)), // arm
          new Pose3d(-0.282, 0.01, 0.305, new Rotation3d(0, intakeAngle, 0)) // intake
        });
  }

  /**
   * @return a double value that smoothly oscilates between 0 and 1, for verifying the RoM of
   *     freshly imported mechanisms
   */
  private double oscilator() {
    return (Math.sin(Timer.getTimestamp()) + 1) / 2;
  }
}
