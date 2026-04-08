package frc.robot.bobot_state.varc;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.bobot_state.BobotState;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants.LeftTrench;
import org.littletonrobotics.FieldConstants.RightTrench;
import org.robotzgarage.frc2026.math.distance_calc.PoseUtils;
import org.robotzgarage.frc2026.math.varc.TargetVARC;

public class TrenchVARC extends TargetVARC {
  private static final Translation2d leftTrench =
      LeftTrench.openingTopLeft.interpolate(LeftTrench.openingTopRight, 0.5).toTranslation2d();
  private static final Translation2d rightTrench =
      RightTrench.openingTopLeft.interpolate(RightTrench.openingTopRight, 0.5).toTranslation2d();

  public void update() {
    Pose2d robotPose = BobotState.getGlobalPose();

    Translation2d robotTranslation = robotPose.getTranslation();
    Distance distanceToLeftTrench =
        Meters.of(robotTranslation.getDistance(AllianceFlipUtil.apply(leftTrench)));
    Distance distanceToRightTrench =
        Meters.of(robotTranslation.getDistance(AllianceFlipUtil.apply(rightTrench)));

    Translation2d trenchTranslation =
        distanceToLeftTrench.lt(distanceToRightTrench) ? leftTrench : rightTrench;

    Pose2d trenchPose = AllianceFlipUtil.apply(new Pose2d(trenchTranslation, Rotation2d.kZero));

    this.distance =
        distanceToLeftTrench.lt(distanceToRightTrench)
            ? distanceToLeftTrench
            : distanceToRightTrench;

    this.rotationTarget =
        PoseUtils.getClosestAngle(
            robotPose.getRotation(),
            // Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(-90),
            Rotation2d.fromDegrees(180));

    // Offset the robot by the length of the intake if we are auto-aligning
    // with the shooter/intake facing parallel of the trench
    double yOffsetIntakeCompensation = 0;
    double rotDeg = this.rotationTarget.getDegrees();
    if (Math.abs(rotDeg - 90) < 1.0) {
      yOffsetIntakeCompensation = Units.inchesToMeters(12);
    } else if (Math.abs(rotDeg + 90) < 1.0) {
      yOffsetIntakeCompensation = -Units.inchesToMeters(12);
    }

    this.targetPose =
        new Pose2d(
            trenchPose.getTranslation().plus(new Translation2d(0, yOffsetIntakeCompensation)),
            trenchPose.getRotation());
    this.rotationTargetPose = new Pose2d(robotTranslation, this.rotationTarget);
  }
}
