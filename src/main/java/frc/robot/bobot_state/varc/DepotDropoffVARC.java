package frc.robot.bobot_state.varc;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.BobotState;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants.Depot;
import org.littletonrobotics.LoggedTunableNumber;
import org.robotzgarage.frc2026.math.distance_calc.PoseUtils;
import org.robotzgarage.frc2026.math.varc.TargetAngleTracker;

public class DepotDropoffVARC extends TargetAngleTracker {
  private static final String name = "/Depot";

  private static final LoggedTunableNumber depotDropoffX =
      new LoggedTunableNumber(name + "/DropoffX", 0.0);
  private static final LoggedTunableNumber depotDropoffY =
      new LoggedTunableNumber(name + "/DropoffY", 7.0);

  public void update() {

    Pose2d robotPose = BobotState.getGlobalPose();
    Pose2d depotPose =
        AllianceFlipUtil.apply(
            PoseUtils.getOffsetPose(
                new Pose2d(Depot.depotCenter.toTranslation2d(), Rotation2d.kZero),
                Feet.of(depotDropoffX.get()),
                Feet.of(depotDropoffY.get())));

    this.distance = Meters.of(depotPose.getTranslation().getDistance(robotPose.getTranslation()));

    this.rotationTarget =
        depotPose.relativeTo(robotPose).getTranslation().getAngle().plus(robotPose.getRotation());

    this.targetPose = depotPose;
  }
}
