package org.robotzgarage.frc2026.math.varc;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import lombok.Getter;

public abstract class TargetAngleTracker {
  @Getter protected Distance distance = Meters.zero();
  @Getter protected Rotation2d rotationTarget = Rotation2d.kZero;

  public abstract void update();
}
