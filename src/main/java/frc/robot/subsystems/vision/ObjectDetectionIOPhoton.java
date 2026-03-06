package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.field.FieldUtils;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.robotzgarage.frc2026.math.distance_calc.PoseUtils;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.ObjectDetectionStrategy;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.ObjectDetectionTrackedPose;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.VisionSource;

public class ObjectDetectionIOPhoton implements ObjectDetectionIO {
  protected final PhotonCamera camera;

  private final Transform3d robotToCamera;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final List<ObjectDetectionStrategy> objectDetectionStrategies;

  public ObjectDetectionIOPhoton(
      VisionSource source,
      Supplier<Pose2d> robotPoseSupplier,
      List<ObjectDetectionStrategy> objectDetectionStrategies) {
    this.camera = new PhotonCamera(source.name());
    this.robotToCamera = source.robotToCamera().getTransform();
    this.robotPoseSupplier = robotPoseSupplier;
    this.objectDetectionStrategies = objectDetectionStrategies;
  }

  public void updateInputs(ObjectDetectionIOInputs inputs) {
    List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();

    List<PhotonTrackedTarget> objectsDetected = new ArrayList<>();

    List<ObjectDetectionTrackedPose> validObjectPoses = new ArrayList<>();
    List<ObjectDetectionTrackedPose> invalidObjectPoses = new ArrayList<>();

    double totalTargetArea = 0.0; // area occupied in frame
    double maxTargetArea = 0.0; // maximum target area occupied by fuel
    double minTargetArea = 0.0; // minimum target area occupied by fuel

    for (PhotonPipelineResult result : unreadResults) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        if (target.getFiducialId() != -1) {
          continue;
        }

        objectsDetected.add(target);
        totalTargetArea += target.area;
        maxTargetArea = Math.max(target.area, maxTargetArea);
        minTargetArea = Math.min(target.area, minTargetArea);

        if (this.objectDetectionStrategies.contains(ObjectDetectionStrategy.POSE_TRACKING)) {
          double xDistanceMeters =
              PhotonUtils.calculateDistanceToTargetMeters(
                  this.robotToCamera.getMeasureZ().in(Meters),
                  VisionConstants.fuelRadiusMeters,
                  -robotToCamera.getRotation().getMeasureY().in(Radians),
                  Units.degreesToRadians(target.getPitch()));

          double distanceToTargetMeters =
              xDistanceMeters / Math.cos(Units.degreesToRadians(target.getYaw()));

          Translation2d cameraToObject =
              PhotonUtils.estimateCameraToTargetTranslation(
                  distanceToTargetMeters, new Rotation2d(-Units.degreesToRadians(target.getYaw())));

          Pose3d objectPose =
              new Pose3d(
                      this.robotPoseSupplier
                          .get()
                          .plus(PoseUtils.transform3dTo2d(this.robotToCamera))
                          .plus(new Transform2d(cameraToObject, Rotation2d.kZero)))
                  .plus(new Transform3d(0, 0, VisionConstants.fuelRadiusMeters, Rotation3d.kZero));

          ObjectDetectionTrackedPose trackedObjectPose =
              new ObjectDetectionTrackedPose(objectPose, result.getTimestampSeconds());

          if (FieldUtils.isPoseInsideField(trackedObjectPose.pose())) {
            validObjectPoses.add(trackedObjectPose);
          } else {
            invalidObjectPoses.add(trackedObjectPose);
          }
        }
      }
    }

    // List<ObjectDetectionTrackedResult> relativeYaws =
    //     objectsDetected.stream()
    //         .map(
    //             targetObject ->
    //                 ObjectDetectionTrackedResult.builder()
    //                     .pitch(Rotation2d.fromDegrees(targetObject.pitch))
    //                     .yaw(Rotation2d.fromDegrees(targetObject.yaw))
    //                     .area(targetObject.area)
    //                     .build())
    //         .toList();

    // Update all inputs at the end
    inputs.connected = this.camera.isConnected();
    inputs.numObjectsInView = objectsDetected.size();
    inputs.validObjectPoses = validObjectPoses.toArray(ObjectDetectionTrackedPose[]::new);
    inputs.invalidObjectPoses = invalidObjectPoses.toArray(ObjectDetectionTrackedPose[]::new);

    if (!objectsDetected.isEmpty()) {
      inputs.avgTargetArea = totalTargetArea / objectsDetected.size();
    } else {
      inputs.avgTargetArea = 0.0;
    }
    inputs.minTargetArea = minTargetArea;
    inputs.maxTargetArea = maxTargetArea;
  }

  @Override
  public List<ObjectDetectionStrategy> getStrategies() {
    return this.objectDetectionStrategies;
  }
}
