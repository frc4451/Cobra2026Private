package frc.robot.bobot_state.varc;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.bobot_state.BobotState;
import lombok.Getter;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants.LeftBump;
import org.littletonrobotics.FieldConstants.RightBump;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.math.varc.TargetAngleTracker;

public class BumpVARC extends TargetAngleTracker {
  @Getter private Translation2d leftBumpCenter = new Translation2d();
  @Getter private Translation2d rightBumpCenter = new Translation2d();

  public void update() {
    leftBumpCenter =
        AllianceFlipUtil.apply(
            LeftBump.nearLeftCorner
                .plus(LeftBump.nearRightCorner)
                .plus(LeftBump.farLeftCorner)
                .plus(LeftBump.farRightCorner)
                .div(4));

    rightBumpCenter =
        AllianceFlipUtil.apply(
            RightBump.nearLeftCorner
                .plus(RightBump.nearRightCorner)
                .plus(RightBump.farLeftCorner)
                .plus(RightBump.farRightCorner)
                .div(4));

    Pose2d robotPose = BobotState.getGlobalPose();
    // Translation2d leftBump =
    //     AllianceFlipUtil.apply(
    //         LeftBump.nearLeftCorner.interpolate((LeftBump.farRightCorner), 0.5));
    // Translation2d rightBump =
    //     AllianceFlipUtil.apply(
    //         RightBump.nearLeftCorner.interpolate((RightBump.farRightCorner), 0.5));
    // Translation2d leftBump =
    // AllianceFlipUtil.apply(LeftBump.nearLeftCorner.plus(LeftBump.nearRightCorner).plus(LeftBump.farLeftCorner).plus(LeftBump.farRightCorner).div(4));
    // Translation2d rightBump =
    // AllianceFlipUtil.apply(RightBump.nearLeftCorner.plus(RightBump.nearRightCorner).plus(RightBump.farLeftCorner).plus(RightBump.farRightCorner).div(4));

    // Translation2d leftBump =
    //     AllianceFlipUtil.apply(LeftBump.nearLeftCorner.interpolate(LeftBump.nearRightCorner,
    // 0.5));
    // Translation2d rightBump =
    //     AllianceFlipUtil.apply(RightBump.nearLeftCorner.interpolate(RightBump.farRightCorner,
    // 0.5));

    Translation2d robotTranslation = robotPose.getTranslation();
    Distance distanceToLeftBump = Meters.of(robotTranslation.getDistance(leftBumpCenter));
    Distance distanceToRightBump = Meters.of(robotTranslation.getDistance(rightBumpCenter));

    boolean leftBumpCloser = distanceToLeftBump.lt(distanceToRightBump);
    // boolean rightBumpCloser = distanceToRightBump.lt(distanceToLeftBump);

    this.distance = leftBumpCloser ? distanceToLeftBump : distanceToRightBump;
    // this.distance = rightBumpCloser ? distanceToRightBump : distanceToLeftBump;

    this.rotationTarget =
        AllianceFlipUtil.apply(
            leftBumpCloser ? Rotation2d.fromDegrees(135) : Rotation2d.fromDegrees(-135));

    // this.distance =
    //     distanceToLeftBump.lt(distanceToRightBump) ? distanceToLeftBump : distanceToRightBump;
    // this.rotationTarget = Rotation2d.fromDegrees(45);

    /*
     * NOTE Ben, you will want to remove these in your next commit
     */
    Logger.recordOutput("BumpVARC/Left/NearLeftCorner", LeftBump.nearLeftCorner);
    Logger.recordOutput("BumpVARC/Left/NearRightCorner", LeftBump.nearRightCorner);
    Logger.recordOutput("BumpVARC/Left/FarLeftCorner", LeftBump.farLeftCorner);
    Logger.recordOutput("BumpVARC/Left/FarRightCorner", LeftBump.farRightCorner);
    Logger.recordOutput(
        "BumpVARC/Left/Average",
        LeftBump.nearLeftCorner
            .plus(LeftBump.nearRightCorner)
            .plus(LeftBump.farLeftCorner)
            .plus(LeftBump.farRightCorner)
            .div(4));

    Logger.recordOutput("BumpVARC/OppLeft/NearLeftCorner", LeftBump.oppNearLeftCorner);
    Logger.recordOutput("BumpVARC/OppLeft/NearRightCorner", LeftBump.oppNearRightCorner);
    Logger.recordOutput("BumpVARC/OppLeft/FarLeftCorner", LeftBump.oppFarLeftCorner);
    Logger.recordOutput("BumpVARC/OppLeft/FarRightCorner", LeftBump.oppFarRightCorner);
    Logger.recordOutput(
        "BumpVARC/OppLeft/Average",
        LeftBump.oppNearLeftCorner
            .plus(LeftBump.oppNearRightCorner)
            .plus(LeftBump.oppFarLeftCorner)
            .plus(LeftBump.oppFarRightCorner)
            .div(4));

    Logger.recordOutput("BumpVARC/Right/NearLeftCorner", RightBump.nearLeftCorner);
    Logger.recordOutput("BumpVARC/Right/NearRightCorner", RightBump.nearRightCorner);
    Logger.recordOutput("BumpVARC/Right/FarLeftCorner", RightBump.farLeftCorner);
    Logger.recordOutput("BumpVARC/Right/FarRightCorner", RightBump.farRightCorner);
    Logger.recordOutput(
        "BumpVARC/Right/Average",
        RightBump.nearLeftCorner
            .plus(RightBump.nearRightCorner)
            .plus(RightBump.farLeftCorner)
            .plus(RightBump.farRightCorner)
            .div(4));
    Logger.recordOutput("BumpVARC/OppRight/NearLeftCorner", RightBump.oppNearLeftCorner);
    Logger.recordOutput("BumpVARC/OppRight/NearRightCorner", RightBump.oppNearRightCorner);
    Logger.recordOutput("BumpVARC/OppRight/FarLeftCorner", RightBump.oppFarLeftCorner);
    Logger.recordOutput("BumpVARC/OppRight/FarRightCorner", RightBump.oppFarRightCorner);
    Logger.recordOutput(
        "BumpVARC/OppRight/Average",
        RightBump.oppNearLeftCorner
            .plus(RightBump.oppNearRightCorner)
            .plus(RightBump.oppFarLeftCorner)
            .plus(RightBump.oppFarRightCorner)
            .div(4));

    Logger.recordOutput("BumpVARC/LeftDistanceFeet", distanceToLeftBump.in(Feet));
    Logger.recordOutput("BumpVARC/RightDistanceFeet", distanceToRightBump.in(Feet));

    Logger.recordOutput("BumpVARC/LeftTranslation", leftBumpCenter);
    Logger.recordOutput("BumpVARC/RightTranslation", rightBumpCenter);
  }
}
