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
    Pose2d robotPose = BobotState.getGlobalPose();

    this.targetPose =
        AllianceFlipUtil.apply(
            new Pose2d(Hub.innerCenterPoint.toTranslation2d(), Hub.nearFace.getRotation()));

    // Do not update this in regard to lookahead. We need the _current_ robot to target
    // distance. Not the projected target to distance. Only aiming matters with
    // shoot-on-the-move
    this.distance =
        Meters.of(this.targetPose.getTranslation().getDistance(robotPose.getTranslation()));

    // If we have a lookahead pose provided to us from Hub Interpolator, we use
    // that over the robot's pose. Otherwise, default to existing robot pose.
    Translation2d lookahead = BobotState.getHubInterpolator().getLookaheadTranslation();

    Pose2d aimPose = lookahead != null ? new Pose2d(lookahead, robotPose.getRotation()) : robotPose;
    this.rotationTarget =
        this.targetPose.relativeTo(aimPose).getTranslation().getAngle().plus(aimPose.getRotation());
  }
}
