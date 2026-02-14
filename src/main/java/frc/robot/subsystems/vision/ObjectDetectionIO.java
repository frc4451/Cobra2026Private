package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.ObjectDetectionTrackedResult;

public interface ObjectDetectionIO {
  @AutoLog
  public static class ObjectDetectionIOInputs {
    public boolean connected = false;
    public int numObjectsInView = 0;
    public ObjectDetectionTrackedResult[] relativeTargets = new ObjectDetectionTrackedResult[0];
  }

  public default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
