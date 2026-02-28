package frc.robot.bobot_state;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.interpolation.HubInterpolator;
import frc.robot.bobot_state.varc.BumpVARC;
import frc.robot.bobot_state.varc.DepotDropoffVARC;
import frc.robot.bobot_state.varc.HubVARC;
import frc.robot.bobot_state.varc.OutpostDropoffVARC;
import frc.robot.bobot_state.varc.TrenchVARC;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import lombok.Getter;
import lombok.Setter;
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
  @Getter private static TargetAngleTracker closestTraversal;
  @Getter private static HubInterpolator hubInterpolator = new HubInterpolator();

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

  public static Queue<PoseObservation> getConstrainedVisionObservations() {
    return BobotState.constrainedPoseObservations;
  }

  // public static TargetAngleTracker getClosestTraversalVARC() {
  //   boolean isBumpCloser = bumpVARC.getDistance().lt(trenchVARC.getDistance());
  //   BobotState.closestTraversal = isBumpCloser ? bumpVARC : trenchVARC;
  //   return closestTraversal;
  // }

  // public static TargetAngleTracker getClosestDropoffVARC() {
  //   boolean isOutpostCloser = outpostVARC.getDistance().lt(depotVARC.getDistance());
  //   BobotState.closestDropoff = isOutpostCloser ? outpostVARC : depotVARC;
  //   return closestDropoff;
  // }

  public void periodic() {
    hubVARC.update();
    trenchVARC.update();
    bumpVARC.update();
    outpostVARC.update();
    depotVARC.update();
    // getClosestDropoffVARC();
    BobotState.hubInterpolator.update(BobotState.globalPose);

    BobotState.closestDropoff =
        outpostVARC.getDistance().lt(depotVARC.getDistance()) ? outpostVARC : depotVARC;

    BobotState.closestTraversal =
        bumpVARC.getDistance().lt(trenchVARC.getDistance()) ? bumpVARC : trenchVARC;

    // {
    //   String subLogTitle = logRoot + "Vision/";
    //   Logger.recordOutput(subLogTitle + "NumGlobalPoses",
    // BobotState.getGlobalPoseObservations().size());
    // }

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

      {
        // hub log
        String varcSubTitle = subLogTitle + "Hub/";

        Logger.recordOutput(varcSubTitle + "TargetPose", hubVARC.getTargetPose());
        Logger.recordOutput(
            varcSubTitle + "TargetRotationDeg", hubVARC.getRotationTarget().getDegrees());
        Logger.recordOutput(varcSubTitle + "TargetDistanceFeet", hubVARC.getDistance().in(Feet));
      }

      {
        // trench log
        String varcSubTitle = subLogTitle + "Trench/";
        Logger.recordOutput(varcSubTitle + "TargetPose", trenchVARC.getTargetPose());
        Logger.recordOutput(
            varcSubTitle + "TargetRotationDeg", trenchVARC.getRotationTarget().getDegrees());
        Logger.recordOutput(varcSubTitle + "TargetDistanceFeet", trenchVARC.getDistance().in(Feet));
      }

      {
        // bump log
        String varcSubTitle = subLogTitle + "Bump/";
        Logger.recordOutput(varcSubTitle + "TargetPose", bumpVARC.getTargetPose());
        Logger.recordOutput(
            varcSubTitle + "TargetRotatinDeg", bumpVARC.getRotationTarget().getDegrees());
        Logger.recordOutput(varcSubTitle + "TargetDistanceFeet", bumpVARC.getDistance().in(Feet));
      }

      {
        // outpost log
        String varcSubTitle = subLogTitle + "Outpost/";
        Logger.recordOutput(varcSubTitle + "TargetPose", outpostVARC.getTargetPose());
        Logger.recordOutput(
            varcSubTitle + "TargetRotationDeg", outpostVARC.getRotationTarget().getDegrees());
        Logger.recordOutput(
            varcSubTitle + "TargetDistanceFeet", outpostVARC.getDistance().in(Feet));
      }

      {
        // depot log
        String varcSubTitle = subLogTitle + "Depot/";
        Logger.recordOutput(varcSubTitle + "TargetPose", depotVARC.getTargetPose());
        Logger.recordOutput(
            varcSubTitle + "TargetRotationDeg", depotVARC.getRotationTarget().getDegrees());
        Logger.recordOutput(varcSubTitle + "TargetDistanceFeet", depotVARC.getDistance().in(Feet));
      }

      {
        // closest traversal log
        String closestSubTitle = subLogTitle + "Traversal/";
        Logger.recordOutput(closestSubTitle + "TargetPose", closestTraversal.getTargetPose());
        Logger.recordOutput(
            closestSubTitle + "TargetRotationDeg",
            closestTraversal.getRotationTarget().getDegrees());
        Logger.recordOutput(
            closestSubTitle + "TargetDistanceFeet", closestTraversal.getDistance().in(Feet));
      }

      {
        // closest dropoff log
        String closestSubTitle = subLogTitle + "Dropoff/";
        Logger.recordOutput(closestSubTitle + "TargetPose", closestDropoff.getTargetPose());
        Logger.recordOutput(
            closestSubTitle + "TargetRotationDeg", closestDropoff.getRotationTarget().getDegrees());
        Logger.recordOutput(
            closestSubTitle + "TargetDistabceFeet", closestDropoff.getDistance().in(Feet));
      }
    }
  }

  public void simulationPeriodic() {}

  // public enum ClosestAlignment {
  //   LeftBump,
  //   RightBump
  // }
}
