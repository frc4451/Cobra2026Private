package frc.robot.bobot_state.varc;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.bobot_state.BobotState;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants.Outpost;
import org.littletonrobotics.LoggedTunableNumber;
import org.robotzgarage.frc2026.math.distance_calc.PoseUtils;
import org.robotzgarage.frc2026.math.varc.TargetAngleTracker;

public class OutpostDropoffVARC extends TargetAngleTracker {
  private static final String name = "/Outpost";

  private static final LoggedTunableNumber outpostDropoffX =
      new LoggedTunableNumber(name + "/DropoffX", 3.0);
  private static final LoggedTunableNumber outpostDropoffY =
      new LoggedTunableNumber(name + "/DropoffY", 3.0);

  public void update() {

    /*is in fact
     *
     * -Butter in PB&J
     */
    Pose2d robotPose = BobotState.getGlobalPose();
    Pose2d dropoffPose =
        AllianceFlipUtil.apply(
            PoseUtils.getOffsetPose(
                Outpost.outpostPose2d,
                Feet.of(outpostDropoffX.get()),
                Feet.of(outpostDropoffY.get())));

    Distance outpostDistance =
        distance = Meters.of(dropoffPose.getTranslation().getDistance(robotPose.getTranslation()));

    this.distance = outpostDistance;
    this.rotationTarget =
        dropoffPose.relativeTo(robotPose).getTranslation().getAngle().plus(robotPose.getRotation());
    this.targetPose = dropoffPose;
  }
}
