package frc.robot.subsystems.PhotonVision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Photon extends SubsystemBase {
  private PhotonIO io;

  private final PhotonIOInputsAutoLogged inputs = new PhotonIOInputsAutoLogged();

  private PhotonCamera camera;

  public Photon(PhotonIO io, String name) {
    this.io = io;
    if (inputs.name != "sim") {
      camera = new PhotonCamera(name);
    }
    Logger.recordOutput("photonresult/name", getName());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Photon" + inputs.name, inputs);
    Logger.recordOutput("photonresult", camera.getAllUnreadResults().size());
  }

  public PhotonPipelineResult result() {
    // if (inputs.name != "sim") {
    return inputs.result;
    // }
    // return new ArrayList<PhotonPipelineResult>();
  }

  public boolean hasTarget() {
    return inputs.hasTargets;
  }

  public boolean hasCoral() {
    Logger.recordOutput("PhotonResult/hasCoral", inputs.hasCoral);
    return inputs.hasCoral;
  }

  public boolean hasAlgae() {
    return inputs.hasAlgae;
  }
}
