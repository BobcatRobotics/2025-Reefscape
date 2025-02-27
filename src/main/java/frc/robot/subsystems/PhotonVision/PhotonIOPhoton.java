package frc.robot.subsystems.PhotonVision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonIOPhoton implements PhotonIO {

  private String name;
  private PhotonCamera camera;
  // private List<PhotonPipelineResult> result;
  private PhotonPipelineResult result;
  private PhotonPipelineResult cacheResult;

  public PhotonIOPhoton(String name) {
    this.name = name;
    camera = new PhotonCamera(name);
    Logger.recordOutput("photonresult/name2", this.name);
  }

  @Override
  public void updateInputs(PhotonIOInputs inputs) {
    // result = camera.getAllUnreadResults();
    inputs.name = name;
    // inputs.hasTargets = hasTargets(inputs.hasTargets);
    // inputs.hasCoral = hasCoral(inputs.hasCoral);
    // inputs.hasAlgae = hasAlgae(inputs.hasAlgae);
    // inputs.result = result(inputs.result);
    result = camera.getLatestResult();
    inputs.hasTargets = result.hasTargets();
    inputs.hasCoral = hasCoral(false);
    inputs.hasAlgae = hasAlgae(false);
    inputs.result = result;

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

  // public PhotonPipelineResult result(PhotonPipelineResult lastVal) {
  //   if (result.isEmpty()) {
  //     return cacheResult;
  //   } else {
  //     cacheResult = result.get(0);
  //     return cacheResult;
  //   }
  // }

  // public boolean hasTargets(boolean lastVal) {
  //   if (result.isEmpty()) {
  //     return lastVal;
  //   } else {
  //     return result.get(0).hasTargets();
  //   }
  // }

  public boolean hasCoral(boolean lastVal) {

    if (result.hasTargets()) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        if (target.objDetectId == 1) {
          return true;
        }
      }
      return false;
    } else {
      return lastVal;
    }
  }

  public boolean hasAlgae(boolean lastVal) {
    if (result.hasTargets()) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        if (target.objDetectId == 0) {
          return true;
        }
      }
      return false;
    } else {
      return lastVal;
    }
  }
}
