package frc.robot.bobot_state.varc;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.bobot_state.BobotState;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants.Hub;
import org.robotzgarage.frc2026.math.varc.TargetAngleTracker;

public class HubVARC extends TargetAngleTracker {
  public void update() {
    Pose2d robotPose = BobotState.getGlobalPose();

    this.targetPose =
        AllianceFlipUtil.apply(
            new Pose2d(Hub.innerCenterPoint.toTranslation2d(), Hub.nearFace.getRotation()));
    this.distance =
        Meters.of(this.targetPose.getTranslation().getDistance(robotPose.getTranslation()));
        
    this.rotationTarget =
        this.targetPose
            .relativeTo(robotPose)
            .getTranslation()
            .getAngle()
            .plus(robotPose.getRotation());
  }
}
