package frc.robot.bobot_state;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants;
import org.littletonrobotics.FieldConstants.LeftTrench;
import org.littletonrobotics.FieldConstants.RightTrench;
import org.littletonrobotics.LoggedTunableNumber;
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

  private static final LoggedTunableNumber neutralZoneFeedingXDistanceFeet =
      new LoggedTunableNumber("NeutralZoneFeedingXDistanceFeet", 5.0);

  private static final LoggedTunableNumber trenchProximityFeet =
      new LoggedTunableNumber("BobotState/TrenchProximityFeet", 4.0);

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

  public static Trigger inNeutralZone() {
    return new Trigger(() -> BobotState.getCurrentFieldZone() == FieldZone.NEUTRAL);
  }

  public static Trigger inFarAllianceZone() {
    return new Trigger(() -> BobotState.getCurrentFieldZone() == FieldZone.FAR_ALLIANCE);
  }

  /**
   * Returns a Trigger that is true when the robot is within {@code trenchProximityMeters} of a
   * trench opening and within the Y gap of that opening.
   *
   * <p>Use {@code .negate()} to allow actions only when clear of the trench.
   */
  public static Trigger isNearTrench() {
    return new Trigger(
        () -> {
          Pose2d robotPose = BobotState.getGlobalPose();
          double robotX = robotPose.getX();
          double robotY = robotPose.getY();
          double proximity = Feet.of(trenchProximityFeet.get()).in(Meters);

          return isInTrenchZone(
                  robotX,
                  robotY,
                  AllianceFlipUtil.apply(LeftTrench.openingTopLeft),
                  AllianceFlipUtil.apply(LeftTrench.openingTopRight),
                  proximity)
              || isInTrenchZone(
                  robotX,
                  robotY,
                  AllianceFlipUtil.apply(LeftTrench.oppOpeningTopLeft),
                  AllianceFlipUtil.apply(LeftTrench.oppOpeningTopRight),
                  proximity)
              || isInTrenchZone(
                  robotX,
                  robotY,
                  AllianceFlipUtil.apply(RightTrench.openingTopLeft),
                  AllianceFlipUtil.apply(RightTrench.openingTopRight),
                  proximity)
              || isInTrenchZone(
                  robotX,
                  robotY,
                  AllianceFlipUtil.apply(RightTrench.oppOpeningTopLeft),
                  AllianceFlipUtil.apply(RightTrench.oppOpeningTopRight),
                  proximity);
        });
  }

  private static boolean isInTrenchZone(
      double robotX,
      double robotY,
      Translation3d openingCorner1,
      Translation3d openingCorner2,
      double proximity) {
    double trenchX = openingCorner1.getX();
    double yMin = Math.min(openingCorner1.getY(), openingCorner2.getY());
    double yMax = Math.max(openingCorner1.getY(), openingCorner2.getY());
    return Math.abs(robotX - trenchX) <= proximity && robotY >= yMin && robotY <= yMax;
  }

  /**
   * Returns a {@link Trigger} that is {@code true} when the robot should stop feeding fuel to the
   * shooter based on its field zone and position relative to the Hub.
   */
  public static Trigger shouldStopFeedingFuelToShooter() {
    return new Trigger(
        () -> {
          // 1) Near Alliance
          if (BobotState.getCurrentFieldZone() == FieldZone.NEAR_ALLIANCE) {
            return false;
          }

          Translation2d shooterTranslation =
              BobotState.getGlobalPose()
                  .transformBy(ShooterConstants.robotToShooter)
                  .getTranslation();

          double shooterX = shooterTranslation.getX();
          double shooterY = shooterTranslation.getY();

          // Hood will be down always when near the trench, this will break shots
          boolean isNearTrench = BobotState.isNearTrench().getAsBoolean();

          double minHubY = FieldConstants.Hub.farRightCorner.getY();
          double maxHubY = FieldConstants.Hub.farLeftCorner.getY();
          boolean shooterInHubYSpan = shooterY >= minHubY && shooterY <= maxHubY;

          // 2) Neutral Zone
          if (BobotState.getCurrentFieldZone() == FieldZone.NEUTRAL) {
            // In the neutral zone the net obstructs shots that are both laterally
            // behind the Hub and far enough past its far face. The X threshold is tunable.
            double hubBlockXThreshold =
                AllianceFlipUtil.applyX(
                    Feet.of(neutralZoneFeedingXDistanceFeet.get()).in(Meters)
                        + FieldConstants.Hub.farFace.getX());
            boolean shooterPastHubFarFaceXThreshold =
                AllianceFlipUtil.shouldFlip()
                    ? shooterX >= hubBlockXThreshold
                    : shooterX <= hubBlockXThreshold;
            return (shooterInHubYSpan && shooterPastHubFarFaceXThreshold) || isNearTrench;
          }

          // 3) Far Alliance
          return shooterInHubYSpan || isNearTrench;
        });
  }

  public static Trigger isTooCloseToHubToScore() {
    return new Trigger(() -> BobotState.hubVARC.getDistance().lt(Feet.of(7.4)))
        .and(BobotState.inNearAllianceZone());
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
      Logger.recordOutput(
          subLogTitle + "ShouldStopFeeding", BobotState.shouldStopFeedingFuelToShooter());

      Logger.recordOutput(
          subLogTitle + "TooCloseToHubToScore", BobotState.isTooCloseToHubToScore());

      Logger.recordOutput(subLogTitle + "IsNearTrench", BobotState.isNearTrench());
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
