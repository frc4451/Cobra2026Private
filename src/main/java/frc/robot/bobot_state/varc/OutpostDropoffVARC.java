package frc.robot.bobot_state.varc;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.bobot_state.BobotState;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants.Outpost;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.math.distance_calc.PoseUtils;
import org.robotzgarage.frc2026.math.varc.TargetAngleTracker;

public class OutpostDropoffVARC extends TargetAngleTracker {
  public void update() {

    /*is in fact
     *
     * -Butter in PB&J
     */
    Pose2d robotPose = BobotState.getGlobalPose();
    Pose2d dropoffPose =
        AllianceFlipUtil.apply(
            PoseUtils.getOffsetPose(Outpost.outpostPose2d, Feet.of(3), Feet.of(3)));

    Distance outpostDistance =
        distance = Meters.of(dropoffPose.getTranslation().getDistance(robotPose.getTranslation()));

    this.distance = outpostDistance;
    this.rotationTarget =
        dropoffPose.relativeTo(robotPose).getTranslation().getAngle().plus(robotPose.getRotation());
    Logger.recordOutput("OutpostVARC/DropoffPose", dropoffPose);

    Logger.recordOutput("OutpostVARC/Distance", outpostDistance);
  }
}
