package org.robotzgarage.frc2026.math.interpolation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import java.util.OptionalDouble;
import java.util.function.Supplier;
import lombok.Builder;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.math.distance_calc.PoseUtils;
import org.robotzgarage.frc2026.math.interpolation.ShootingInterpolator.InterpolatedCalculation;

public abstract class TargetInterpolator {
  private final ShootingInterpolator interpolator;

  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> robotVelocitySupplier;
  @Getter private final Supplier<Pose2d> targetPose;
  @Getter private InterpolatedCalculation calculation;
  private final int lookaheadIterations;
  // This is instanced as a primative type to allow it to be nullable
  private final double phaseDelaySeconds;
  private final String logKey;

  @Builder
  public record TargetInterpolatorConfig(
      String logKey,
      Supplier<Pose2d> targetPose,
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> robotVelocity,
      ShootingInterpolator interpolator,
      int lookaheadIterations,
      Time phaseDelay) {}

  public TargetInterpolator(TargetInterpolatorConfig config) {
    this.robotPose = config.robotPose();
    this.robotVelocitySupplier = config.robotVelocity();
    this.targetPose = config.targetPose();
    this.interpolator = config.interpolator();
    this.lookaheadIterations = config.lookaheadIterations();
    this.phaseDelaySeconds = config.phaseDelay() != null ? config.phaseDelay().in(Seconds) : 0.0;
    this.logKey = config.logKey() != null ? config.logKey() + "/" : "Interpolation/";
    this.calculation =
        new InterpolatedCalculation(
            Degrees.zero(), RotationsPerSecond.zero(), RotationsPerSecond.zero(), null);
  }

  public void update() {
    double currentDistanceMeters = getDistanceFromTarget().in(Meters);
    double lookaheadDistanceMeters = currentDistanceMeters;

    OptionalDouble initialTOF = interpolator.getTOF(lookaheadDistanceMeters);
    if (initialTOF.isPresent()) {
      ChassisSpeeds fieldVelocity = robotVelocitySupplier.get();
      Translation2d robotTranslation = robotPose.get().getTranslation();
      Translation2d target = targetPose.get().getTranslation();

      // Iteratively solve for lookahead distance.
      // Use the current TOF estimate to project where the robot will be when
      // fuel lands in the hub, then recalculate the distance from that projected position.
      Pose2d currentPose = robotPose.get();
      Translation2d lookaheadTranslation = robotTranslation;
      for (int i = 0; i < lookaheadIterations; i++) {
        double timeOfFlight =
            interpolator.getTOF(lookaheadDistanceMeters).getAsDouble() + phaseDelaySeconds;
        lookaheadTranslation =
            robotTranslation.plus(
                new Translation2d(
                    fieldVelocity.vxMetersPerSecond * timeOfFlight,
                    fieldVelocity.vyMetersPerSecond * timeOfFlight));
        lookaheadDistanceMeters = lookaheadTranslation.getDistance(target);
      }

      Logger.recordOutput(
          logKey + "LookaheadPose", new Pose2d(lookaheadTranslation, currentPose.getRotation()));
      Logger.recordOutput(
          logKey + "LookaheadDistanceFeet", Units.metersToFeet(lookaheadDistanceMeters));
      Logger.recordOutput(
          logKey + "CurrentDistanceFeet", Units.metersToFeet(currentDistanceMeters));
      Logger.recordOutput(
          logKey + "LookaheadCorrectionFeet",
          Units.metersToFeet(lookaheadDistanceMeters - currentDistanceMeters));
    }

    this.calculation = interpolator.calculateInterpolation(Meters.of(lookaheadDistanceMeters));
  }

  /**
   * Calculate the distance between the robot and the target we want to aim at
   *
   * @return hypotenuse distance between the robot and the target
   */
  public Distance getDistanceFromTarget() {
    return PoseUtils.getDistanceBetweenPoses(robotPose.get(), targetPose.get());
  }
}
