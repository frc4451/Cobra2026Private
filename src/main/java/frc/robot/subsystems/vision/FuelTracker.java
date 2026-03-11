package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.LoggedTunableNumber;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.ObjectDetectionTrackedPose;

public class FuelTracker {
  private static final LoggedTunableNumber deduplicationThresholdMeters =
      new LoggedTunableNumber("ObjectDetection/FuelTracker/DeduplicationThresholdMeters", 0.4);

  private static final LoggedTunableNumber trackingMemorySeconds =
      new LoggedTunableNumber(
          "ObjectDetection/FuelTracker/TrackingMemorySeconds",
          VisionConstants.trackingMemorySeconds);

  // One entry per distinct fuel ball, not per observation frame
  private final List<ObjectDetectionTrackedPose> trackedPoses = new ArrayList<>();

  /** Every robot loop, evict any tracked fuel that hasn't been seen within the tracking window. */
  public void update() {
    double cutoff = Timer.getFPGATimestamp() - trackingMemorySeconds.get();
    trackedPoses.removeIf(p -> p.timestampSeconds() < cutoff);
  }

  /**
   * Add new detections, merging any that are within the deduplication threshold of an existing
   * tracked pose. A spatial match refreshes the existing entry with the newer pose and timestamp
   * rather than inserting a duplicate.
   */
  public void addMoreTrackedPoses(List<ObjectDetectionTrackedPose> newTrackedPoses) {
    double threshold = deduplicationThresholdMeters.get();

    for (ObjectDetectionTrackedPose newPose : newTrackedPoses) {
      boolean merged = false;

      for (int i = 0; i < trackedPoses.size(); i++) {
        double distance =
            newPose
                .pose()
                .getTranslation()
                .toTranslation2d()
                .getDistance(trackedPoses.get(i).pose().getTranslation().toTranslation2d());

        if (distance < threshold) {
          trackedPoses.set(i, newPose);
          merged = true;
          break;
        }
      }

      if (!merged) {
        trackedPoses.add(newPose);
      }
    }
  }

  /** Utility method for "just getting poses" */
  public List<Pose3d> getObjectPoses() {
    return trackedPoses.stream().map(ObjectDetectionTrackedPose::pose).toList();
  }
}
