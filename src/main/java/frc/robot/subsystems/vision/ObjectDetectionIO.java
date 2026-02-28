package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.ObjectDetectionStrategy;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.ObjectDetectionTrackedPose;

public interface ObjectDetectionIO {
  @AutoLog
  public static class ObjectDetectionIOInputs {
    public boolean connected = false;
    public int numObjectsInView = 0;
    public ObjectDetectionTrackedPose[] validObjectPoses = new ObjectDetectionTrackedPose[0];
    public ObjectDetectionTrackedPose[] invalidObjectPoses = new ObjectDetectionTrackedPose[0];
    public double avgTargetArea;
    public double maxTargetArea;
    public double minTargetArea;
  }

  public default void updateInputs(ObjectDetectionIOInputs inputs) {}

  public default List<ObjectDetectionStrategy> getStrategies() {
    return new ArrayList<>();
  }
}
