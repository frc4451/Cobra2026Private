package frc.robot.subsystems.superstructure.intakeComponent;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.superstructure.shooterComponent.shooterComponentConstan.HoodConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.robotzgarage.frc2026.subsystems.pivot.PivotMechanism;

public class PivotingMechanism {
  private final String key;
  private final LoggedMechanism2d mechanism;
  private final PivotMechanism pivotMechanism;

  public PivotingMechanism(String key, PivotMechanism pivotMechanism) {
    this.key = key;
    this.pivotMechanism = pivotMechanism;

    mechanism =
        new LoggedMechanism2d(
            HoodConstants.rootMechanismConfig.displayWidth(),
            HoodConstants.rootMechanismConfig.displayHeight(),
            new Color8Bit(Color.kWhite));

    LoggedMechanismRoot2d root =
        mechanism.getRoot(
            "PivotingMechanism",
            HoodConstants.rootMechanismConfig.rootPosition().getX(),
            HoodConstants.rootMechanismConfig.rootPosition().getY());

    root.append(pivotMechanism.getPivotMech());
  }

  public void update(Angle newPivotAngle) {
    pivotMechanism.update(newPivotAngle);
    Logger.recordOutput(key, mechanism);
  }
}
