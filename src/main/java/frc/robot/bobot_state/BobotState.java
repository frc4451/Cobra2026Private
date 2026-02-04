package frc.robot.bobot_state;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.varc.HubVARC;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants.Hub;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.PoseObservation;
import org.robotzgarage.frc2026.util.VirtualSubsystem;

public class BobotState extends VirtualSubsystem {
  private static final String logRoot = "BobotState/";

  @Getter private static final HubVARC hubVARC = new HubVARC();

  @Getter
  private static final Queue<PoseObservation> globalPoseObservations =
      new LinkedBlockingQueue<>(20);

  @Getter
  private static final Queue<PoseObservation> trigPoseObservations = new LinkedBlockingQueue<>(20);

  @Getter
  private static final Queue<PoseObservation> constrainedPoseObservations =
      new LinkedBlockingQueue<>(20);

  @Getter @Setter private static Pose2d globalPose = new Pose2d();
  @Getter @Setter private static Pose2d trigPose = new Pose2d();
  @Getter @Setter private static Pose2d constrainedPose = new Pose2d();
  @Getter @Setter private static Rotation2d rawGyroHeading = new Rotation2d();

  public static void offerGlobalVisionObservation(PoseObservation observation) {
    BobotState.globalPoseObservations.offer(observation);
  }

  public static void offerTrigVisionObservation(PoseObservation observation) {
    BobotState.trigPoseObservations.offer(observation);
  }

  public static void offerConstrainedVisionObservation(PoseObservation observation) {
    BobotState.constrainedPoseObservations.offer(observation);
  }

  public void periodic() {
    hubVARC.update();

    {
      String subLogTitle = logRoot + "Interpolation/";
      Logger.recordOutput(
          subLogTitle + "DistanceFromCenterOfHub",
          Meters.of(
                  Hub.innerCenterPoint
                      .toTranslation2d()
                      .getDistance(BobotState.globalPose.getTranslation()))
              .in(Feet));
    }

    {
      String subLogTitle = logRoot + "VARC/";
      Logger.recordOutput(
          subLogTitle + "Hub/TargetPose", AllianceFlipUtil.apply(Hub.innerCenterPoint));
      Logger.recordOutput(
          subLogTitle + "Hub/TargetRotationDeg", hubVARC.getRotationTarget().getDegrees());
      Logger.recordOutput(subLogTitle + "Hub/TargetDistanceFeet", hubVARC.getDistance().in(Feet));
    }
  }

  public void simulationPeriodic() {}
}
