package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.FieldConstants;

public class FieldUtils {
  public static boolean isPoseInsideField(Pose2d target) {
    return target.getX() > 0.0
        && target.getX() < FieldConstants.fieldLength
        && target.getY() > 0.0
        && target.getY() < FieldConstants.fieldWidth;
  }

  public static boolean isPoseInsideField(Pose3d target) {
    return FieldUtils.isPoseInsideField(target.toPose2d());
  }
}
