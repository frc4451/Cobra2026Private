package org.robotzgarage.frc2026.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.robotzgarage.frc2026.subsystems.vision.AprilTagAlgorithms.SingleTagAlgorithms;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.PoseEstimationMethod;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.PoseObservation;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.VisionSource;

public class AprilTagIOPhoton implements AprilTagIO {
  protected final PhotonCamera camera;

  /**
   * In 2026, PhotonVision uses a single pose estimator with helper methods to handle individual
   * strategies
   */
  protected final PhotonPoseEstimator poseEstimator;

  /** This should compute our localized pose to targets we care about */
  // private final PhotonPoseEstimator constrainedEstimator;

  private final Supplier<Rotation2d> headingSupplier;

  private final List<PoseEstimationMethod> supportedPoseEstimationMethods;

  public AprilTagIOPhoton(
      VisionSource source,
      AprilTagFieldLayout aprilTagFieldLayout,
      Supplier<Rotation2d> headingSupplier,
      List<PoseEstimationMethod> supportedPoseEstimationMethods) {
    camera = new PhotonCamera(source.name());

    poseEstimator =
        new PhotonPoseEstimator(aprilTagFieldLayout, source.robotToCamera().getTransform());

    this.headingSupplier = headingSupplier;
    this.supportedPoseEstimationMethods = supportedPoseEstimationMethods;
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();

    List<Translation2d> validCorners = new ArrayList<>();
    List<Translation2d> rejectedCorners = new ArrayList<>();

    List<Integer> validIds = new ArrayList<>();
    List<Integer> rejectedIds = new ArrayList<>();

    List<PoseObservation> validPoseObservations = new ArrayList<>();
    List<PoseObservation> rejectedPoseObservations = new ArrayList<>();

    List<Pose3d> validPoses = new ArrayList<>();
    List<Pose3d> rejectedPoses = new ArrayList<>();

    List<Pose3d> validAprilTagPoses = new ArrayList<>();
    List<Pose3d> rejectedAprilTagPoses = new ArrayList<>();

    for (PhotonPipelineResult result : unreadResults) {
      // Detected Corners
      for (PhotonTrackedTarget target : result.getTargets()) {
        if (SingleTagAlgorithms.isUsable(target)) {
          target.getDetectedCorners().stream()
              .map(corner -> new Translation2d(corner.x, corner.y))
              .forEach(validCorners::add);

          validIds.add(target.getFiducialId());

          validAprilTagPoses.add(
              poseEstimator.getFieldTags().getTagPose(target.getFiducialId()).get());
        } else {
          target.getDetectedCorners().stream()
              .map(corner -> new Translation2d(corner.x, corner.y))
              .forEach(rejectedCorners::add);

          rejectedIds.add(target.getFiducialId());

          if (target.getFiducialId() != -1) {
            poseEstimator
                .getFieldTags()
                .getTagPose(target.getFiducialId())
                .ifPresent(rejectedAprilTagPoses::add);
          }
        }
      }

      /*************************************************************************
       * Primary Vision strategy should be to use MultiTag PNP on Coprocessor,
       * however, we should fall back to Lowest Ambiguity on single tag results.
       * If no results estimates are available, skip this loop.
       *
       * If a user specifies `CONSTRAINED`, you _have_ to compute MULTI_TAG.
       */
      if (supportedPoseEstimationMethods.contains(PoseEstimationMethod.MULTI_TAG)
          || supportedPoseEstimationMethods.contains(PoseEstimationMethod.CONSTRAINED)) {
        Optional<EstimatedRobotPose> maybeEstimatedMultiTagPose =
            poseEstimator.estimateCoprocMultiTagPose(result);
        Optional<EstimatedRobotPose> maybeEstimatedLowestAmbiguity =
            poseEstimator.estimateLowestAmbiguityPose(result);

        // If both results are empty skip the robot loop
        if (maybeEstimatedMultiTagPose.isEmpty() && maybeEstimatedLowestAmbiguity.isEmpty()) {
          continue;
        }

        // Assume that if we made it here, that we have at least lowest ambiguity
        EstimatedRobotPose estimatedGlobalPose =
            maybeEstimatedMultiTagPose.orElse(maybeEstimatedLowestAmbiguity.get());

        if (result.getMultiTagResult().isPresent()) {
          MultiTargetPNPResult multiTagResult = result.getMultiTagResult().get();

          Pose3d globalPose = estimatedGlobalPose.estimatedPose;
          Matrix<N3, N1> stdDevs =
              AprilTagAlgorithms.getEstimationStdDevs(globalPose.toPose2d(), result.getTargets());

          PoseObservation observation =
              new PoseObservation(
                  estimatedGlobalPose.estimatedPose,
                  estimatedGlobalPose.timestampSeconds,
                  multiTagResult.estimatedPose.ambiguity,
                  -100,
                  stdDevs,
                  PoseEstimationMethod.MULTI_TAG);

          validPoseObservations.add(observation);
          validPoses.add(observation.robotPose());

          for (PhotonTrackedTarget target : result.getTargets()) {
            target.getDetectedCorners().stream()
                .map(corner -> new Translation2d(corner.x, corner.y))
                .forEach(validCorners::add);

            validIds.add(target.getFiducialId());

            validAprilTagPoses.add(
                poseEstimator.getFieldTags().getTagPose(target.getFiducialId()).get());
          }
        } else if (!result.getTargets().isEmpty()) {
          PhotonTrackedTarget target = result.getTargets().get(0);

          Pose3d pose = estimatedGlobalPose.estimatedPose;
          Matrix<N3, N1> stdDevs =
              AprilTagAlgorithms.getEstimationStdDevs(pose.toPose2d(), result.getTargets());
          PoseObservation observation =
              new PoseObservation(
                  pose,
                  estimatedGlobalPose.timestampSeconds,
                  target.poseAmbiguity,
                  target.fiducialId,
                  stdDevs,
                  PoseEstimationMethod.SINGLE_TAG);

          if (SingleTagAlgorithms.isUsable(target)) {
            validPoseObservations.add(observation);
            validPoses.add(observation.robotPose());
          } else {
            rejectedPoseObservations.add(observation);
            rejectedPoses.add(observation.robotPose());
          }
        }

        /*************************************************************************
         * We may additionally want to use CONSTRAINED for refinement.
         * In order to use CONSTRAINED, you need to supply a "truth" for robot
         * heading and an initial guess of the robot pose. This is typically the
         * IMU of the robot, and your `MULTITAG` result, respectively.
         * Make the IMU heading a `Supplier<Rotation2d>` and pass it with
         * `headingSupplier`.
         */
        if (supportedPoseEstimationMethods.contains(PoseEstimationMethod.CONSTRAINED)) {
          poseEstimator.addHeadingData(result.getTimestampSeconds(), headingSupplier.get());

          /*
           * We have to provide camera intrinsics and distortion from Network Tables
           * to give the RIO the information needed to compute CONSTRAINED_SOLVEPNP.
           */
          Optional<EstimatedRobotPose> maybeConstrainedPose =
              poseEstimator.estimateConstrainedSolvepnpPose(
                  result,
                  camera.getCameraMatrix().get(),
                  camera.getDistCoeffs().get(),
                  estimatedGlobalPose.estimatedPose,
                  true,
                  0);

          if (!maybeConstrainedPose.isPresent()) {
            continue;
          }

          EstimatedRobotPose estimatedConstrainedPose = maybeConstrainedPose.get();

          Matrix<N3, N1> constrainedStdDevs =
              AprilTagAlgorithms.getEstimationStdDevs(
                  estimatedConstrainedPose.estimatedPose.toPose2d(), result.getTargets());

          PoseObservation constrainedObservation =
              new PoseObservation(
                  estimatedConstrainedPose.estimatedPose,
                  estimatedConstrainedPose.timestampSeconds,
                  -100, // constrained observations use gyro heading as validation
                  result.getBestTarget().getFiducialId(),
                  constrainedStdDevs,
                  PoseEstimationMethod.CONSTRAINED);

          validPoseObservations.add(constrainedObservation);
          validPoses.add(constrainedObservation.robotPose());
        }
      }
      if (supportedPoseEstimationMethods.contains(PoseEstimationMethod.TRIG)) {
        poseEstimator.addHeadingData(result.getTimestampSeconds(), headingSupplier.get());

        /*
         * We have to provide camera intrinsics and distortion from Network Tables
         * to give the RIO the information needed to compute CONSTRAINED_SOLVEPNP.
         */
        Optional<EstimatedRobotPose> maybeTrigPose =
            poseEstimator.estimatePnpDistanceTrigSolvePose(result);

        if (!maybeTrigPose.isPresent()) {
          continue;
        }

        EstimatedRobotPose estimatedTrigPose = maybeTrigPose.get();

        Matrix<N3, N1> constrainedStdDevs =
            AprilTagAlgorithms.getEstimationStdDevs(
                estimatedTrigPose.estimatedPose.toPose2d(), result.getTargets());

        PoseObservation constrainedObservation =
            new PoseObservation(
                estimatedTrigPose.estimatedPose,
                estimatedTrigPose.timestampSeconds,
                -100, // trig observations use gyro heading as validation
                result.getBestTarget().getFiducialId(),
                constrainedStdDevs,
                PoseEstimationMethod.TRIG);

        validPoseObservations.add(constrainedObservation);
        validPoses.add(constrainedObservation.robotPose());
      }
    }

    inputs.connected = camera.isConnected();

    inputs.validCorners = validCorners.toArray(Translation2d[]::new);
    inputs.rejectedCorners = rejectedCorners.toArray(Translation2d[]::new);

    inputs.validIds = validIds.stream().mapToInt(Integer::intValue).toArray();
    inputs.rejectedIds = rejectedIds.stream().mapToInt(Integer::intValue).toArray();

    inputs.validPoseObservations = validPoseObservations.toArray(PoseObservation[]::new);
    inputs.rejectedPoseObservations = rejectedPoseObservations.toArray(PoseObservation[]::new);

    inputs.validPoses = validPoses.toArray(Pose3d[]::new);
    inputs.rejectedPoses = rejectedPoses.toArray(Pose3d[]::new);

    inputs.validAprilTagPoses = validAprilTagPoses.toArray(Pose3d[]::new);
    inputs.rejectedAprilTagPoses = rejectedAprilTagPoses.toArray(Pose3d[]::new);
  }
}
