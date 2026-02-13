package frc.robot.bobot_state;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants;

public class FieldState {
  public enum FieldZone {
    NEAR,
    NEUTRAL,
    FAR
  }

  public static FieldZone getFieldZone() {
    double fieldRelativeX =
        AllianceFlipUtil.applyX(BobotState.getGlobalPose().getTranslation().getX());

    if (fieldRelativeX < FieldConstants.LinesVertical.neutralZoneNear) {
      return FieldZone.NEAR;
    } else if (fieldRelativeX < FieldConstants.LinesVertical.neutralZoneFar) {
      return FieldZone.NEUTRAL;
    } else {
      return FieldZone.FAR;
    }
  }

  public static Trigger isInNearAllianceZone() {
    return new Trigger(() -> FieldState.getFieldZone() == FieldZone.NEAR);
  }

  public static Trigger isInNeutralZone() {
    return new Trigger(() -> FieldState.getFieldZone() == FieldZone.NEUTRAL);
  }

  public static Trigger isInOpponentAllianceZone() {
    return new Trigger(() -> FieldState.getFieldZone() == FieldZone.FAR);
  }
}
