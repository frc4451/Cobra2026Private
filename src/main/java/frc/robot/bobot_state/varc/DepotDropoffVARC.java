package frc.robot.bobot_state.varc;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.bobot_state.BobotState;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants;
import org.littletonrobotics.LoggedTunableNumber;
import org.robotzgarage.frc2026.math.distance_calc.PoseUtils;
import org.robotzgarage.frc2026.math.varc.TargetVARC;

public class DepotDropoffVARC extends TargetVARC {
  public static final LoggedTunableNumber parallelOffsetFeet =
      new LoggedTunableNumber("DepotDropoffParallelOffsetFeet", 19.0);
  public static final LoggedTunableNumber perpendicularOffsetFeet =
      new LoggedTunableNumber("DepotDropoffPerpendicularOffsetFeet", 6.0);

  public void update() {
    Pose2d robotPose = BobotState.getGlobalPose();
    // Pose2d depotPose =
    //     AllianceFlipUtil.apply(
    //         PoseUtils.getOffsetPose(
    //             new Pose2d(Depot.depotCenter.toTranslation2d(), Rotation2d.kZero),
    //             Feet.of(parallelOffsetFeet.get()),
    //             Feet.of(perpendicularOffsetFeet.get())));

    Pose2d depotPose =
        AllianceFlipUtil.apply(
            PoseUtils.getOffsetPose(
                FieldConstants.origin,
                Feet.of(parallelOffsetFeet.get()),
                Feet.of(perpendicularOffsetFeet.get())));

    this.distance = Meters.of(depotPose.getTranslation().getDistance(robotPose.getTranslation()));

    this.rotationTarget =
        depotPose.relativeTo(robotPose).getTranslation().getAngle().plus(robotPose.getRotation());

    this.targetPose = depotPose;
  }
}
