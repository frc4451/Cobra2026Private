package frc.robot.bobot_state;

import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.bobot_state.interpolation.DropoffInterpolator;
import frc.robot.bobot_state.interpolation.HubInterpolator;
import frc.robot.bobot_state.varc.BumpVARC;
import frc.robot.bobot_state.varc.DepotDropoffVARC;
import frc.robot.bobot_state.varc.FuelClusterVARC;
import frc.robot.bobot_state.varc.HubVARC;
import frc.robot.bobot_state.varc.OutpostDropoffVARC;
import frc.robot.bobot_state.varc.TrenchVARC;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.math.interpolation.TargetInterpolator;
import org.robotzgarage.frc2026.math.varc.TargetVARC;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.PoseObservation;
import org.robotzgarage.frc2026.util.VirtualSubsystem;

public class BobotState extends VirtualSubsystem {
  private static final String logRoot = "BobotState/";

  public enum FieldZone {
    NEAR_ALLIANCE,
    NEUTRAL,
    FAR_ALLIANCE,
    OUTSIDE
  }

  @Getter private static final HubVARC hubVARC = new HubVARC();
  @Getter private static final TrenchVARC trenchVARC = new TrenchVARC();
  @Getter private static final BumpVARC bumpVARC = new BumpVARC();
  @Getter private static final OutpostDropoffVARC outpostVARC = new OutpostDropoffVARC();
  @Getter private static final DepotDropoffVARC depotVARC = new DepotDropoffVARC();
  @Getter private static final FuelClusterVARC fuelClusterVARC = new FuelClusterVARC();
  @Getter private static TargetVARC closestDropoff = outpostVARC;
  @Getter private static TargetVARC closestTraversal;

  @Getter @Setter private static List<Pose3d> knownFuelPoses = List.of();
  @Getter private static HubInterpolator hubInterpolator = new HubInterpolator();
  @Getter private static DropoffInterpolator dropoffInterpolator = new DropoffInterpolator();
  // @gETTER private static

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
  @Getter @Setter private static ChassisSpeeds robotVelocity = new ChassisSpeeds();

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

  public static FieldZone getCurrentFieldZone() {
    double currentXDistance = BobotState.getGlobalPose().getX();

    double nearHubX = AllianceFlipUtil.applyX(FieldConstants.Hub.centerOfHub.getX());
    double farHubX = AllianceFlipUtil.applyX(FieldConstants.Hub.oppCenterOfHub.getX());

    // For Red alliance, applyX flips the coordinates so nearHubX > farHubX.
    // Sort them so comparisons work correctly for both alliances.
    double minX = Math.min(nearHubX, farHubX);
    double maxX = Math.max(nearHubX, farHubX);

    if (currentXDistance <= minX) {
      return AllianceFlipUtil.shouldFlip() ? FieldZone.FAR_ALLIANCE : FieldZone.NEAR_ALLIANCE;
    } else if (currentXDistance <= maxX) {
      return FieldZone.NEUTRAL;
    } else {
      return AllianceFlipUtil.shouldFlip() ? FieldZone.NEAR_ALLIANCE : FieldZone.FAR_ALLIANCE;
    }
  }

  /**
   * Return the VARC needed for the drive base, controlled by the driver, for where the shooter
   * should face at any time. This should be read by
   *
   * @return
   */
  public static TargetVARC getShooterAlignmentVARC() {
    return switch (BobotState.getCurrentFieldZone()) {
      case NEAR_ALLIANCE -> BobotState.getHubVARC();
      default -> BobotState.getClosestDropoff();
    };
  }

  public static TargetInterpolator getShooterInterpolator() {
    return switch (BobotState.getCurrentFieldZone()) {
      case NEAR_ALLIANCE -> BobotState.hubInterpolator;
      default -> BobotState.dropoffInterpolator;
    };
  }

  public static Trigger inNearAllianceZone() {
    return new Trigger(() -> BobotState.getCurrentFieldZone() == FieldZone.NEAR_ALLIANCE);
  }

  public void periodic() {
    // VARCs must update before the hub interpolator so that HubVARC can read
    // hubInterpolator.getLookaheadTranslation() from the previous cycle. The one-loop
    // (~20 ms) staleness is acceptable. Do NOT move hubInterpolator.update() above the VARCs.
    hubVARC.update();
    trenchVARC.update();
    bumpVARC.update();
    outpostVARC.update();
    depotVARC.update();
    fuelClusterVARC.update();
    // getClosestDropoffVARC();
    BobotState.hubInterpolator.update();
    BobotState.dropoffInterpolator.update();

    BobotState.closestDropoff =
        outpostVARC.getDistance().lt(depotVARC.getDistance()) ? outpostVARC : depotVARC;

    BobotState.closestTraversal =
        bumpVARC.getDistance().lt(trenchVARC.getDistance()) ? bumpVARC : trenchVARC;

    {
      String subLogTitle = logRoot + "Zoning/";
      Logger.recordOutput(subLogTitle + "CurrentZone", BobotState.getCurrentFieldZone());
      Logger.recordOutput(
          subLogTitle + "CurrentVARCTarget",
          BobotState.getCurrentFieldZone() == FieldZone.NEAR_ALLIANCE
              ? "HUB"
              : BobotState.getClosestDropoff() instanceof DepotDropoffVARC ? "DEPOT" : "OUTPOST");
    }

    // {
    //   String subLogTitle = logRoot + "Vision/";
    //   Logger.recordOutput(subLogTitle + "NumGlobalPoses",
    // BobotState.getGlobalPoseObservations().size());
    // }

    {
      String subLogTitle = logRoot + "Interpolation/";
      Logger.recordOutput(subLogTitle + "RobotVelocity", BobotState.getRobotVelocity());
      Logger.recordOutput(
          subLogTitle + "DistanceFromCenterOfHubFeet",
          BobotState.getHubVARC().getDistance().in(Feet));
      Logger.recordOutput(
          subLogTitle + "DistanceFromOutpostDropoffFeet",
          BobotState.getOutpostVARC().getDistance().in(Feet));
      Logger.recordOutput(
          subLogTitle + "DistanceFromDepotDropoffFeet",
          BobotState.getDepotVARC().getDistance().in(Feet));
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
            closestSubTitle + "ClosestTraversal",
            closestTraversal instanceof BumpVARC ? "BUMP" : "TRENCH");
        Logger.recordOutput(
            closestSubTitle + "TargetRotationDeg",
            closestTraversal.getRotationTarget().getDegrees());
        Logger.recordOutput(
            closestSubTitle + "TargetDistanceFeet", closestTraversal.getDistance().in(Feet));
      }

      {
        // closest dropoff log
        String closestSubTitle = subLogTitle + "Dropoff/";
        Logger.recordOutput(
            closestSubTitle + "ClosestDropoff",
            closestDropoff instanceof DepotDropoffVARC ? "DEPOT" : "OUTPOST");
        Logger.recordOutput(closestSubTitle + "TargetPose", closestDropoff.getTargetPose());
        Logger.recordOutput(
            closestSubTitle + "TargetRotationDeg", closestDropoff.getRotationTarget().getDegrees());
        Logger.recordOutput(
            closestSubTitle + "TargetDistabceFeet", closestDropoff.getDistance().in(Feet));
      }

      {
        // fuel cluster log
        String varcSubTitle = subLogTitle + "FuelCluster/";
        Logger.recordOutput(varcSubTitle + "TargetPose", fuelClusterVARC.getTargetPose());
        Logger.recordOutput(
            varcSubTitle + "TargetRotationDeg", fuelClusterVARC.getRotationTarget().getDegrees());
        Logger.recordOutput(
            varcSubTitle + "TargetDistanceFeet", fuelClusterVARC.getDistance().in(Feet));
      }
    }
  }

  public void simulationPeriodic() {}

  // public enum ClosestAlignment {
  //   LeftBump,
  //   RightBump
  // }
}
