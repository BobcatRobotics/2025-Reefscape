package frc.robot.subsystems.PhotonVision;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonIOPhoton implements PhotonIO {

  private String name;
  private PhotonCamera camera;
  private List<PhotonPipelineResult> result;

  public PhotonIOPhoton(String name) {
    this.name = name;
    camera = new PhotonCamera(name);
    result = camera.getAllUnreadResults();
  }

  @Override
  public void updateInputs(PhotonIOInputs inputs) {

    inputs.name = name;
    inputs.hasTargets = result.get(0).hasTargets();
    inputs.hasCoral = hasCoral();
    inputs.hasAlgae = hasAlgae();
    // inputs.target = result.getTargets();
    // public boolean hasTargets = false;
    // public int classID;
    // public double confidence;
    // public double area;
    // public double pitch;
    // public double yaw;

    // public List<TargetCorner> minAreaRectCorners;

    // // Corners from whatever corner detection method was used
    // public List<TargetCorner> detectedCorners;

  }

  public boolean hasCoral() {

    for (PhotonTrackedTarget target : result.get(0).getTargets()) {
      if (target.objDetectId == 1) {
        return true;
      }
    }
    return false;
  }

  public boolean hasAlgae() {

    for (PhotonTrackedTarget target : result.get(1).getTargets()) {
      if (target.objDetectId == 1) {
        return true;
      }
    }
    return false;
  }
}
