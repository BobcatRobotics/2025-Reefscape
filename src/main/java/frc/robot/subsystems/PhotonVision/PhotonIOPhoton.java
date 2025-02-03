package frc.robot.subsystems.PhotonVision;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonIOPhoton implements PhotonIO {

  private String name;
  private PhotonCamera camera;
  private List<PhotonPipelineResult> result;

  public PhotonIOPhoton(String name) {
    name = this.name;
    camera = new PhotonCamera(name);
    result = camera.getAllUnreadResults();
  }

  @Override
  public void updateInputs(PhotonIOInputs inputs) {

    inputs.name = name;
    inputs.hasTargets = result.get(0).hasTargets();
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
}
