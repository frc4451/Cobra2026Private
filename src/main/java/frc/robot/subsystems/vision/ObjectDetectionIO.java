package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {
  @AutoLog
  public static class ObjectDetectionIOInputs {
    public boolean connected = false;
    public int numObjectsInView = 0;
    // public ObjectDetectionTrackedResult[] relativeTargets = new ObjecyDetectionTrackedResult[0];
  }

  public default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
