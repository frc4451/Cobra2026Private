package frc.robot.bobot_state.varc;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.bobot_state.BobotState;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants.LeftBump;
import org.littletonrobotics.FieldConstants.RightBump;
import org.robotzgarage.frc2026.math.varc.TargetAngleTracker;

public class BumpVARC extends TargetAngleTracker {
  private static final Translation2d leftBumpCenter = AllianceFlipUtil.apply(LeftBump.bumpCenter);
  private static final Translation2d rightBumpCenter = AllianceFlipUtil.apply(RightBump.bumpCenter);

  public void update() {
    Pose2d robotPose = BobotState.getGlobalPose();

    Translation2d robotTranslation = robotPose.getTranslation();
    Distance distanceToLeftBump = Meters.of(robotTranslation.getDistance(leftBumpCenter));
    Distance distanceToRightBump = Meters.of(robotTranslation.getDistance(rightBumpCenter));

    Translation2d bumpTranslation =
        distanceToLeftBump.lt(distanceToRightBump) ? leftBumpCenter : rightBumpCenter;

    Pose2d bumpPose =
        AllianceFlipUtil.apply(
            new Pose2d(AllianceFlipUtil.apply(bumpTranslation), Rotation2d.kZero));

    boolean leftBumpCloser = distanceToLeftBump.lt(distanceToRightBump);

    this.distance = leftBumpCloser ? distanceToLeftBump : distanceToRightBump;

    // this.rotationTarget = (leftBumpCloser ? VARCConstants.leftBumpRotation :
    // VARCConstants.rightBumpRotation)
    //         .getValue();
    this.rotationTarget =
        closestAngle(
            robotPose.getRotation(),
            Rotation2d.fromDegrees(45),
            Rotation2d.fromDegrees(-45),
            Rotation2d.fromDegrees(135),
            Rotation2d.fromDegrees(-135));

    this.targetPose = bumpPose;
    this.rotationTargetPose = new Pose2d(robotTranslation, this.rotationTarget);
  }
}
