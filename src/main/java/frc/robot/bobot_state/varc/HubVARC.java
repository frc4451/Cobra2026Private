package frc.robot.bobot_state.varc;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.bobot_state.BobotState;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants.Hub;
import org.robotzgarage.frc2026.math.varc.TargetAngleTracker;

public class HubVARC extends TargetAngleTracker {
  public void update() {
    Pose2d robotBose = BobotState.getGlobalPose();
    Translation2d hubLocation = AllianceFlipUtil.apply(Hub.innerCenterPoint).toTranslation2d();
    Pose2d centerOfHubWithNearFaceRotation =
        AllianceFlipUtil.apply(
            new Pose2d(Hub.innerCenterPoint.toTranslation2d(), Hub.nearFace.getRotation()));

    this.distance = Meters.of(hubLocation.getDistance(robotBose.getTranslation()));
    this.rotationTarget =
        centerOfHubWithNearFaceRotation
            .relativeTo(robotBose)
            .getTranslation()
            .getAngle()
            .plus(robotBose.getRotation());
  }
}
