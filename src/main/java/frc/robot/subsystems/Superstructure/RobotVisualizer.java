package frc.robot.subsystems.Superstructure;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class RobotVisualizer {
  // the main mechanism object
  LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);
  // the mechanism root node
  LoggedMechanismRoot2d root = mech.getRoot("climber", 2, 0);

  // MechanismLigament2d objects represent each "section"/"stage" of the
  // mechanism, and are based off the root node or another ligament object
  LoggedMechanismLigament2d m_elevator =
      root.append(new LoggedMechanismLigament2d("elevator", 1, 90));
  LoggedMechanismLigament2d m_wrist =
      m_elevator.append(
          new LoggedMechanismLigament2d("wrist", 0.5, 0, 6, new Color8Bit(Color.kPurple)));

  public RobotVisualizer() {
    Logger.recordOutput("Mech", mech);
  }
}
