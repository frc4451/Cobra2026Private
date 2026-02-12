package frc.robot.bobot_state;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.varc.BumpVARC;
import frc.robot.bobot_state.varc.DepotDropoffVARC;
import frc.robot.bobot_state.varc.HubVARC;
import frc.robot.bobot_state.varc.OutpostDropoffVARC;
import frc.robot.bobot_state.varc.TrenchVARC;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants.Hub;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.math.varc.TargetAngleTracker;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.PoseObservation;
import org.robotzgarage.frc2026.util.VirtualSubsystem;

public class BobotState extends VirtualSubsystem {
  private static final String logRoot = "BobotState/";

  @Getter private static final HubVARC hubVARC = new HubVARC();
  @Getter private static final TrenchVARC trenchVARC = new TrenchVARC();
  @Getter private static final BumpVARC bumpVARC = new BumpVARC();
  @Getter private static final OutpostDropoffVARC outpostVARC = new OutpostDropoffVARC();
  @Getter private static final DepotDropoffVARC depotVARC = new DepotDropoffVARC();
  @Getter private static TargetAngleTracker closestDropoff;

  private static final Queue<PoseObservation> globalPoseObservations =
      new LinkedBlockingQueue<>(20);
  private static final Queue<PoseObservation> constrainedPoseObservations =
      new LinkedBlockingQueue<>(20);

  @Getter @Setter private static Pose2d globalPose = new Pose2d();
  @Getter @Setter private static Pose2d constrainedPose = new Pose2d();
  @Getter @Setter private static Rotation2d rawGyroHeading = new Rotation2d();

  public static void offerGlobalVisionObservation(PoseObservation observation) {
    BobotState.globalPoseObservations.offer(observation);
  }

  public static Queue<PoseObservation> getGlobalVisionObservations() {
    return BobotState.globalPoseObservations;
  }

  public static void offerConstrainedVisionObservation(PoseObservation observation) {
    BobotState.constrainedPoseObservations.offer(observation);
  }

  public static Queue<PoseObservation> getConstrainedVisionObservations() {
    return BobotState.constrainedPoseObservations;
  }

  public static TargetAngleTracker getClosestTraversalVARC() {
    boolean isBumpCloser = bumpVARC.getDistance().lt(trenchVARC.getDistance());
    return isBumpCloser ? bumpVARC : trenchVARC;
  }

  public static TargetAngleTracker getClosestDropoffVARC() {
    boolean isOutpostCloser = outpostVARC.getDistance().lt(depotVARC.getDistance());
    BobotState.closestDropoff = isOutpostCloser ? outpostVARC : depotVARC;
    return closestDropoff;
  }

  public void periodic() {
    hubVARC.update();
    trenchVARC.update();
    bumpVARC.update();
    outpostVARC.update();
    depotVARC.update();
    getClosestDropoffVARC();

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
      Logger.recordOutput(
          subLogTitle + "Trench/TargetDistanceFeet", trenchVARC.getDistance().in(Feet));
      Logger.recordOutput(
          subLogTitle + "Trench/TargetRotationDeg", trenchVARC.getRotationTarget().getDegrees());
      Logger.recordOutput(subLogTitle + "Bump/TargetDistanceFeet", bumpVARC.getDistance().in(Feet));
      Logger.recordOutput(
          subLogTitle + "Outpost/TargetAngleDeg", outpostVARC.getRotationTarget().getDegrees());
      Logger.recordOutput(subLogTitle + "Outpost/TargetPose", outpostVARC.getTargetPose());
      Logger.recordOutput(subLogTitle + "Depot/TargetPose", depotVARC.getTargetPose());
      Logger.recordOutput(
          subLogTitle + "ClosestDropoff/TargetPose", closestDropoff.getTargetPose());
    }
  }

  public void simulationPeriodic() {}

  public enum ClosestAlignment {
    LeftBump,
    RightBump
  }
}
