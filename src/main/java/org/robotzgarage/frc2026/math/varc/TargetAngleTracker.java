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

  public abstract void update();

  public static Rotation2d closestAngle(Rotation2d currentPose, Rotation2d... targetangles) {
    Rotation2d closest = Rotation2d.fromDegrees(360);

    // for (Rotation2d target : targetangles) {
    //   if (Math.abs(currentPose.getDegrees() - target.getDegrees()) < closest.getDegrees()) {
    //     closest = target;
    //   }
    // }

    for (Rotation2d target : targetangles) {
      if (Math.abs(currentPose.relativeTo(target).getDegrees()) < closest.getDegrees()) {
        closest = target;
      }
    }

    return closest;
  }
}
