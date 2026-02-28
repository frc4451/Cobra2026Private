package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.ObjectDetectionTrackedPose;

public class FuelTracker {
  //   private List<ObjectDetectionTrackedPose> objectDetectionTrackedPoses = new ArrayList<>();

  // Prefer a PriorityQueue over Lists because they always give preference to the "most recent"
  // objects when seeking
  private final PriorityQueue<ObjectDetectionTrackedPose> trackedPoses =
      new PriorityQueue<>(Comparator.comparingDouble(ObjectDetectionTrackedPose::timestampSeconds));

  /**
   * Every robot loop, check that all tracked Fuel are within our "tracking" threshold. If it
   * exceeds our threshold, remove them from the heap and move on.
   */
  public void update() {
    double now = Timer.getFPGATimestamp();
    double cutoff = now - VisionConstants.trackingMemorySeconds;

    while (!trackedPoses.isEmpty() && trackedPoses.peek().timestampSeconds() < cutoff) {
      trackedPoses.poll();
    }
  }

  //   public void update() {
  //     // Remove objects if the last time we saw them was more than `trackingMemorySeconds`
  //     objectDetectionTrackedPoses.removeIf(
  //         object -> Timer.getFPGATimestamp() - object.timestampSeconds() >
  // VisionConstants.trackingMemorySeconds
  //     );
  //   }

  public void addMoreTrackedPoses(List<ObjectDetectionTrackedPose> newTrackedPoses) {
    // this.objectDetectionTrackedPoses.addAll(newTrackedPoses);
    this.trackedPoses.addAll(newTrackedPoses);
  }

  /** Utility method for "just getting poses" */
  public List<Pose3d> getObjectPoses() {
    return this.trackedPoses.stream().map(trackedPose -> trackedPose.pose()).toList();
  }
}
