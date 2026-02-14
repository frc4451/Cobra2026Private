package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.ObjectDetectionTrackedResult;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.VisionSource;

public class ObjectDetectionIOPhoton implements ObjectDetectionIO {
  protected final PhotonCamera camera;

  public ObjectDetectionIOPhoton(VisionSource source) {
    this.camera = new PhotonCamera(source.name());
  }

  public void updateInputs(ObjectDetectionIOInputs inputs) {
    List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();

    List<PhotonTrackedTarget> objectsDetected = new ArrayList<>();

    for (PhotonPipelineResult result : unreadResults) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        if (target.getFiducialId() == -1) {
          objectsDetected.add(target);
        }
      }
    }

    List<ObjectDetectionTrackedResult> relativeYaws =
        objectsDetected.stream()
            .map(
                targetObject ->
                    ObjectDetectionTrackedResult.builder()
                        .pitch(Rotation2d.fromDegrees(targetObject.pitch))
                        .yaw(Rotation2d.fromDegrees(targetObject.yaw))
                        .area(targetObject.area)
                        .build())
            .toList();

    // Update all inputs at the end
    inputs.connected = this.camera.isConnected();
    inputs.numObjectsInView = relativeYaws.size();
    inputs.relativeTargets = relativeYaws.toArray(ObjectDetectionTrackedResult[]::new);
  }
}
