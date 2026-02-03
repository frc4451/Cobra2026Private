package frc.robot.bobot_state.varc;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.bobot_state.BobotState;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants.LeftBump;
import org.littletonrobotics.FieldConstants.RightBump;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.math.varc.TargetAngleTracker;

public class BumpVARC extends TargetAngleTracker {
  public void update() {
    Pose2d robotPose = BobotState.getGlobalPose();
    // Translation2d leftBump =
    //     AllianceFlipUtil.apply(
    //         LeftBump.nearLeftCorner.interpolate((LeftBump.farRightCorner), 0.5));
    // Translation2d rightBump =
    //     AllianceFlipUtil.apply(
    //         RightBump.nearLeftCorner.interpolate((RightBump.farRightCorner), 0.5));
    // Translation2d leftBump = AllianceFlipUtil.apply(LeftBump.nearLeftCorner.plus(LeftBump.nearRightCorner).plus(LeftBump.farLeftCorner).plus(LeftBump.farRightCorner).div(4));
    // Translation2d rightBump = AllianceFlipUtil.apply(RightBump.nearLeftCorner.plus(RightBump.nearRightCorner).plus(RightBump.farLeftCorner).plus(RightBump.farRightCorner).div(4));

    Translation2d leftBump = AllianceFlipUtil.apply(LeftBump.nearLeftCorner.interpolate(LeftBump.farRightCorner, 0.5));
    Translation2d rightBump = AllianceFlipUtil.apply(RightBump.nearLeftCorner.interpolate(RightBump.farRightCorner,0.5));

    Translation2d robotTranslation = robotPose.getTranslation();
    Distance distanceToLeftBump = Meters.of(robotTranslation.getDistance(leftBump));
    Distance distanceToRightBump = Meters.of(robotTranslation.getDistance(rightBump));

    this.distance =
        distanceToLeftBump.lt(distanceToRightBump) ? distanceToLeftBump : distanceToRightBump;
    this.rotationTarget = Rotation2d.fromDegrees(45);

    Logger.recordOutput("BumpVARC/LeftDistanceFeet", distanceToLeftBump.in(Feet));
    Logger.recordOutput("BumpVARC/RightDistanceFeet", distanceToRightBump.in(Feet));

    Logger.recordOutput("BumpVARC/LeftTranslation", leftBump);
    Logger.recordOutput("BumpVARC/RightTranslation", rightBump);
  }
}
