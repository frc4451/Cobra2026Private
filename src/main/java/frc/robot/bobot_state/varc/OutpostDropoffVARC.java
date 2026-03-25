package frc.robot.bobot_state.varc;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.bobot_state.BobotState;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants;
import org.littletonrobotics.LoggedTunableNumber;
import org.robotzgarage.frc2026.math.distance_calc.PoseUtils;
import org.robotzgarage.frc2026.math.varc.TargetVARC;

public class OutpostDropoffVARC extends TargetVARC {
  public static final LoggedTunableNumber parallelOffsetFeet =
      new LoggedTunableNumber("OutpostDropoffParallelOffsetFeet", 3.0);
  public static final LoggedTunableNumber perpendicularOffsetFeet =
      new LoggedTunableNumber("OutpostDropoffPerpendicularOffsetFeet", 6.0);

  public void update() {

    /*is in fact
     *
     * -Butter in PB&J
     */
    Pose2d robotPose = BobotState.getGlobalPose();
    Pose2d dropoffPose =
        AllianceFlipUtil.apply(
            PoseUtils.getOffsetPose(
                FieldConstants.origin,
                Feet.of(parallelOffsetFeet.get()),
                Feet.of(perpendicularOffsetFeet.get())));

    Distance outpostDistance =
        distance = Meters.of(dropoffPose.getTranslation().getDistance(robotPose.getTranslation()));

    this.distance = outpostDistance;
    this.rotationTarget =
        dropoffPose.relativeTo(robotPose).getTranslation().getAngle().plus(robotPose.getRotation());
    this.targetPose = dropoffPose;
  }
}
