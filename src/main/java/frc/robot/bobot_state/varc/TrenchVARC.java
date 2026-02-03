package frc.robot.bobot_state.varc;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.bobot_state.BobotState;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants.LeftTrench;
import org.littletonrobotics.FieldConstants.RightTrench;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.math.varc.TargetAngleTracker;

public class TrenchVARC extends TargetAngleTracker {
  public void update() {
    Pose2d robotPose = BobotState.getGlobalPose();
    Translation2d leftTrench =
        AllianceFlipUtil.apply(
            LeftTrench.openingTopLeft
                .interpolate(LeftTrench.openingTopRight, 0.5)
                .toTranslation2d());
    Translation2d rightTrench =
        AllianceFlipUtil.apply(
            RightTrench.openingTopLeft
                .interpolate(RightTrench.openingTopRight, 0.5)
                .toTranslation2d());

    Logger.recordOutput("TrenchVARC/LeftTrench", leftTrench);
    Logger.recordOutput("TrenchVARC/RightTrench", rightTrench);

    Translation2d robotTranslation = robotPose.getTranslation();
    Distance distanceToLeftTrench = Meters.of(robotTranslation.getDistance(leftTrench));
    Distance distanceToRightTrench = Meters.of(robotTranslation.getDistance(rightTrench));

    Logger.recordOutput("TrenchVARC/LeftDistanceFeet", distanceToLeftTrench.in(Feet));
    Logger.recordOutput("TrenchVARC/RightDistanceFeet", distanceToRightTrench.in(Feet));

    this.distance =
        distanceToLeftTrench.lt(distanceToRightTrench)
            ? distanceToLeftTrench
            : distanceToRightTrench;
  }
}
