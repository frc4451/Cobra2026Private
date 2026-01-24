package org.robotzgarage.frc2026.math.interpolation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import lombok.Getter;
import org.robotzgarage.frc2026.math.distance_calc.PoseUtils;
import org.robotzgarage.frc2026.math.interpolation.ShootingInterpolator.InterpolatedCalculation;

public abstract class TargetInterpolator {
  @Getter private final Pose2d targetPose;
  private final ShootingInterpolator interpolator;

  private Pose2d robotPose;
  @Getter private InterpolatedCalculation calculation;

  public TargetInterpolator(Pose2d targetPose, ShootingInterpolator interpolator) {
    this.robotPose = new Pose2d();
    this.targetPose = targetPose;
    this.interpolator = interpolator;
    this.calculation =
        new InterpolatedCalculation(
            Degrees.zero(), RotationsPerSecond.zero(), RotationsPerSecond.zero());
  }

  public void update(Pose2d robotPose) {
    this.robotPose = robotPose;
    this.calculation = interpolator.calculateInterpolation(getDistanceFromTarget());
  }

  /**
   * Calculate the distance between the robot and the target we want to aim at
   *
   * @return hypotenuse distance between the robot and the target
   */
  public Distance getDistanceFromTarget() {
    return PoseUtils.getDistanceBetweenPoses(robotPose, targetPose);
  }
}
