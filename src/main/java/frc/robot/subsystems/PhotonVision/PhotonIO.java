package frc.robot.subsystems.PhotonVision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

public interface PhotonIO {

  /** The set of loggable inputs for the vision subsystem. */
  @AutoLog
  public static class PhotonIOInputs {
    public String name = "Sim";
    public boolean hasTargets = false;
    public boolean hasCoral = false;
    public boolean hasAlgae = false;
    public PhotonPipelineResult result = new PhotonPipelineResult();
    // public Array[] targets;
    // public List<PhotonTrackedTarget> target = new ArrayList<PhotonTrackedTarget>();
    // public int classID;
    // public double confidence;
    // public double area;
    // public double pitch;
    // public double yaw;

    // public List<TargetCorner> minAreaRectCorners;

    // // Corners from whatever corner detection method was used
    // public List<TargetCorner> detectedCorners;
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(PhotonIOInputs inputs) {}
}
