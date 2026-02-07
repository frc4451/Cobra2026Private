package frc.robot.bobot_state.varc;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.bobot_state.BobotState;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants.LeftBump;
import org.littletonrobotics.FieldConstants.RightBump;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.math.varc.TargetAngleTracker;

public class BumpVARC extends TargetAngleTracker {
  private static final Translation2d leftBumpCenter = AllianceFlipUtil.apply(LeftBump.bumpCenter);
  private static final Translation2d rightBumpCenter = AllianceFlipUtil.apply(RightBump.bumpCenter);

  public void update() {
    Pose2d robotPose = BobotState.getGlobalPose();

    Translation2d robotTranslation = robotPose.getTranslation();
    Distance distanceToLeftBump = Meters.of(robotTranslation.getDistance(leftBumpCenter));
    Distance distanceToRightBump = Meters.of(robotTranslation.getDistance(rightBumpCenter));

    boolean leftBumpCloser = distanceToLeftBump.lt(distanceToRightBump);

    this.distance = leftBumpCloser ? distanceToLeftBump : distanceToRightBump;

    this.rotationTarget =
        AllianceFlipUtil.apply(
            leftBumpCloser ? ConstantsVARC.leftBumpRotation : ConstantsVARC.rightBumpRotation);

    Logger.recordOutput("BumpVARC/LeftDistanceFeet", distanceToLeftBump.in(Feet));
    Logger.recordOutput("BumpVARC/RightDistanceFeet", distanceToRightBump.in(Feet));

    Logger.recordOutput("BumpVARC/LeftTranslation", leftBumpCenter);
    Logger.recordOutput("BumpVARC/RightTranslation", rightBumpCenter);
  }
}
