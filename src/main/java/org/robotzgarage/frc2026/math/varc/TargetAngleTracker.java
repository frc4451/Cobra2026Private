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
}
