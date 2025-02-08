// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.util.VisionObservation.LLTYPE;
import org.littletonrobotics.junction.AutoLog;

/** Vision subsystem hardware interface. */
public interface VisionIO {
  /** The set of loggable inputs for the vision subsystem. */
  @AutoLog
  public static class VisionIOInputs {
    public LEDMode ledMode = LEDMode.FORCEOFF;
    public double pipelineID = 0;
    public double pipelineLatency = 0;
    public double ta = -1;
    public boolean tv = false;
    public double tx = -1;
    public double ty = -1;
    public double fiducialID = -1;
    public double tClass = -1;
    public String name = "sim";
    // public CamMode camMode = CamMode.VISION;
    public Pose2d botPoseMG2 = new Pose2d();
    public int tagCount = -1;
    public double avgTagDist = -1;
    public Pose3d botPose3d = new Pose3d();
    public double timestamp = -1;
    public LLTYPE limelightType;
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  /** Sets the pipeline number. */
  public default void setLEDS(LEDMode mode) {}

  public default void setPipeline(String limelight, int index) {}

  public default double getTClass() {
    return 0.0;
  }

  public default void setCamMode(CamMode mode) {}

  public default void setRobotOrientationMG2(Rotation3d gyro, Rotation3d rate) {}

  public default void setPermittedTags(int[] tags) {}

  public default void setPriorityID(int tagID) {}
}
