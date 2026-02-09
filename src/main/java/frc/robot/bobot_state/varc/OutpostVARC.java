package frc.robot.bobot_state.varc;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.bobot_state.BobotState;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants.Outpost;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.math.varc.TargetAngleTracker;

public class OutpostVARC extends TargetAngleTracker {
  public void update() {

    /*is in fact, 
     * extremely,
     * unoquivocally,
     * and most definietly... likely,
     * broken.
     * 
     * -Butter in PB&J
     */
    Pose2d robotPose = BobotState.getGlobalPose();
    Translation2d outpostLocation = AllianceFlipUtil.apply(Outpost.centerPoint);
    // Pose2d outpostPose = AllianceFlipUtil.apply(Outpost.outpostPose2d);

    Logger.recordOutput("OutpostVARC/Location", outpostLocation);
    Distance outpostDistance = distance = Meters.of(outpostLocation.getDistance(robotPose.getTranslation()));
    this.distance = outpostDistance;
    this.rotationTarget = AllianceFlipUtil.apply(Rotation2d.fromDegrees(0));
    // outpostPose.relativeTo(robotPose).getTranslation().getAngle().plus(robotPose.getRotation());

    Logger.recordOutput("OutpostVARC/Distance", outpostDistance);
  }
}
