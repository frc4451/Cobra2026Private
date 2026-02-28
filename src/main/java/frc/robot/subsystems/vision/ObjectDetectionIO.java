package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {
  @AutoLog
  public static class ObjectDetectionIOInputs {
    public boolean connected = false;
    public int numObjectsInView = 0;
    public Pose3d[] validObjectPoses = new Pose3d[0];
    public Pose3d[] invalidObjectPoses = new Pose3d[0];
    public double avgTargetArea;
    public double maxTargetArea;
    public double minTargetArea;
  }

  public default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
