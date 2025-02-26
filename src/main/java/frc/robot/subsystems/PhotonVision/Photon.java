package frc.robot.subsystems.PhotonVision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Photon extends SubsystemBase {
  private PhotonIO io;

  private final PhotonIOInputsAutoLogged inputs = new PhotonIOInputsAutoLogged();

  private PhotonCamera camera;

  public Photon(PhotonIO io) {
    this.io = io;
    if (inputs.name != "sim") {
      camera = new PhotonCamera(inputs.name);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Photon" + inputs.name, inputs);
  }

  public List<PhotonPipelineResult> result() {
    if (inputs.name != "sim") {
      return camera.getAllUnreadResults();
    }
    return new ArrayList<PhotonPipelineResult>();
  }

  public boolean hasTarget() {
    return inputs.hasTargets;
  }

  public boolean hasCoral() {
    return inputs.hasCoral;
  }

  public boolean hasAlgae() {
    return inputs.hasAlgae;
  }
}
