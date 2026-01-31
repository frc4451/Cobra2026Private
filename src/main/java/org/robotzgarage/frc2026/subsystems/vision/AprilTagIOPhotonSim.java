package org.robotzgarage.frc2026.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.SimCameraConfig;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.VisionSource;

public class AprilTagIOPhotonSim extends AprilTagIOPhoton {
  private final PhotonCameraSim cameraSim;

  public AprilTagIOPhotonSim(
      VisionSource source,
      AprilTagFieldLayout aprilTagFieldLayout,
      Supplier<Rotation2d> headingSupplier,
      boolean calculateConstrainedSolvePNP,
      SimCameraConfig config) {
    super(source, aprilTagFieldLayout, headingSupplier, calculateConstrainedSolvePNP);

    SimCameraProperties props = config.apply(new SimCameraProperties());

    cameraSim = new PhotonCameraSim(camera, props, aprilTagFieldLayout);

    cameraSim.enableDrawWireframe(true);
    cameraSim.setMaxSightRange(10.0);
    cameraSim.setWireframeResolution(1);

    VisionConstants.aprilTagSim.ifPresent(
        aprilTagSim -> aprilTagSim.addCamera(cameraSim, source.robotToCamera().getTransform()));
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    super.updateInputs(inputs);

    VisionConstants.aprilTagSim.ifPresent(
        aprilTagSim -> {
          FieldObject2d visionEstimation =
              aprilTagSim.getDebugField().getObject("VisionEstimation");

          if (inputs.validPoseObservations.length != 0) {
            visionEstimation.setPoses(inputs.validPoseObservations[0].robotPose().toPose2d());
          } else {
            visionEstimation.setPoses();
          }
        });
  }
}
