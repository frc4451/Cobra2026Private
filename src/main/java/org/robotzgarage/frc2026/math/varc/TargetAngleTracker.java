package org.robotzgarage.frc2026.math.varc;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import lombok.Getter;

public abstract class TargetAngleTracker {
  @Getter protected Distance distance = Meters.zero();
  @Getter protected Rotation2d rotationTarget = Rotation2d.kZero;
  @Getter protected Pose2d targetPose;
  @Getter protected Pose2d rotationTargetPose;

  public abstract void update();

  public static Rotation2d closestAngle(Rotation2d current, Rotation2d... targets) {
    Rotation2d closest = Rotation2d.fromDegrees(Double.MAX_VALUE);
    Rotation2d result = Rotation2d.fromDegrees(Double.MAX_VALUE);

    for (Rotation2d target : targets) {
      if (Math.abs(target.minus(current).getDegrees()) < closest.getDegrees()) {
        closest = Rotation2d.fromDegrees(Math.abs(target.minus(current).getDegrees()));
        result = target;
      }
    }

    // for (Rotation2d target : targetangles) {
    //   if (Math.abs(currentPose.relativeTo(target).getDegrees()) < closest.getDegrees()) {
    //     closest = target;
    //   }
    // }

    return result;
  }
}
