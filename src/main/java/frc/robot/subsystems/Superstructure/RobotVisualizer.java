package frc.robot.subsystems.Superstructure;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {
  // Singleton instance
  private static RobotVisualizer instance;

  private double elevatorHeightMeters = 0;
  private double armAngleRadians = 0;

  private RobotVisualizer() {
  }

  // Singleton accessor
  public static RobotVisualizer getInstance() {
    if (instance == null) {
      instance = new RobotVisualizer();
    }
    return instance;
  }



  public void setElevatorHeight(Distance height) {
    // Consider adding bounds checking if necessary.
    elevatorHeightMeters = height.in(Meters);
    update();
  }

  /**
   * 0 deg is horizontal pointing right
   */
  public void setArmRotation(Rotation2d angle) {
    armAngleRadians = angle.getRadians();
    update();
  }

  private void update() {
    //the distance between each stage of the elevator
    double stageHeight = elevatorHeightMeters/3;
    Logger.recordOutput(
        "Visualization/Mechanisms",
        new Pose3d[] {
          new Pose3d(0, -0.192, 0.14 + stageHeight, new Rotation3d()), // stage 1
          new Pose3d(0, -0.192, 0.16 + stageHeight * 2, new Rotation3d()), // stage 2
          new Pose3d(0, -0.192, 0.18 + stageHeight * 3, new Rotation3d()), // carrige
          new Pose3d(
            0, -0.21, 0.313 + stageHeight * 3,
             new Rotation3d(
              0,
              armAngleRadians,
              0
             )) // arm
        });
  }
}
