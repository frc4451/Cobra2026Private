package frc.robot.bobot_state.varc;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.BobotState;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants.Depot;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.math.distance_calc.PoseUtils;
import org.robotzgarage.frc2026.math.varc.TargetAngleTracker;

public class DepotDropoffVARC extends TargetAngleTracker {
  public void update() {

    Pose2d robotPose = BobotState.getGlobalPose();
    Pose2d depotPose =
        AllianceFlipUtil.apply(
            PoseUtils.getOffsetPose(
                new Pose2d(Depot.depotCenter.toTranslation2d(), Rotation2d.kZero),
                Feet.of(0),
                Feet.of(7)));

    this.distance = Meters.of(depotPose.getTranslation().getDistance(robotPose.getTranslation()));

    this.rotationTarget =
        depotPose.relativeTo(robotPose).getTranslation().getAngle().plus(robotPose.getRotation());

    Logger.recordOutput("DepotVARC/DepotPose", depotPose);

    Logger.recordOutput("DepotVARC/Distance", distance);
  }
}
