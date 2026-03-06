package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.ObjectDetectionStrategy;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.SimCameraConfig;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.VisionSource;

public class ObjectDetectionIOPhotonSim extends ObjectDetectionIOPhoton {
  private final PhotonCameraSim cameraSim;

  public ObjectDetectionIOPhotonSim(
      VisionSource source,
      Supplier<Pose2d> robotPoseSupplier,
      List<ObjectDetectionStrategy> objectDetectionStrategies,
      SimCameraConfig config) {
    super(source, robotPoseSupplier, objectDetectionStrategies);

    SimCameraProperties props = config.apply(new SimCameraProperties());
    cameraSim = new PhotonCameraSim(camera, props);
    cameraSim.enableDrawWireframe(true);
    cameraSim.setMaxSightRange(10.0);
    cameraSim.setWireframeResolution(1);

    VisionConstants.objectDetectionSim.ifPresent(
        objectDetectionSim ->
            objectDetectionSim.addCamera(cameraSim, source.robotToCamera().getTransform()));
  }
}
