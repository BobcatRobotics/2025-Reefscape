package frc.robot.subsystems.EndEffector;

public class EndEffectorIOSim implements EndEffectorIO {
  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.hasPiece = true;
  }
}
