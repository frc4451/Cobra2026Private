package org.robotzgarage.frc2026.math.interpolation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import java.util.Optional;
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
  private final Supplier<Transform2d> robotToShooter;
  @Getter private final Supplier<Pose2d> targetPose;
  @Getter private InterpolatedCalculation calculation;
  /** Converged shooter position at the moment the ball is predicted to reach the target. */
  @Getter private Translation2d lookaheadTranslation = null;

  // Hysteresis state for the overshoot guard — prevents rapid flipping between
  // SOTM-adjusted aim and direct aim when near the dot=0 boundary.
  private boolean wasOvershot = false;

  private final int lookaheadIterations;
  // Stored as a primitive double because it is allowed to be zero (no delay).
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
      Time phaseDelay,
      Supplier<Transform2d> robotToShooter) {}

  public TargetInterpolator(TargetInterpolatorConfig config) {
    this.robotPose = config.robotPose();
    this.robotVelocitySupplier = config.robotVelocity();
    this.targetPose = config.targetPose();
    this.interpolator = config.interpolator();
    this.lookaheadIterations = config.lookaheadIterations();
    this.phaseDelaySeconds = config.phaseDelay() != null ? config.phaseDelay().in(Seconds) : 0.0;
    this.logKey = config.logKey() != null ? config.logKey() + "/" : "Interpolation/";
    this.robotToShooter = config.robotToShooter();
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
      Pose2d currentPose = robotPose.get();
      Translation2d shooterTranslation =
          robotToShooter != null
              ? currentPose.transformBy(robotToShooter.get()).getTranslation()
              : currentPose.getTranslation();
      Translation2d target = targetPose.get().getTranslation();

      // Iteratively solve for lookahead distance.
      // Use the current TOF estimate to project where the shooter will be when
      // fuel lands in the hub, then recalculate the distance from that projected position.
      Translation2d lookaheadTranslation = shooterTranslation;
      for (int i = 0; i < lookaheadIterations; i++) {
        double timeOfFlight =
            interpolator.getTOF(lookaheadDistanceMeters).getAsDouble() + phaseDelaySeconds;
        lookaheadTranslation =
            shooterTranslation.plus(
                new Translation2d(
                    fieldVelocity.vxMetersPerSecond * timeOfFlight,
                    fieldVelocity.vyMetersPerSecond * timeOfFlight));
        lookaheadDistanceMeters = lookaheadTranslation.getDistance(target);
      }

      // If the converged lookahead has overshot past the target (robot moving toward
      // target faster than ball flight time allows), the aim angle would flip ~180°. Detect
      // this with a dot-product sign check and fall back to direct aim in that case.
      Translation2d currentToTarget = target.minus(shooterTranslation);
      Translation2d lookaheadToTarget = target.minus(lookaheadTranslation);
      double dot =
          currentToTarget.getX() * lookaheadToTarget.getX()
              + currentToTarget.getY() * lookaheadToTarget.getY();
      
      // Enter overshot when dot <= 0; don't exit until dot exceeds 10% of
      // |currentToTarget|^2 so the flag doesn't chatter at the boundary and cause PID spikes.
      double exitThreshold =
          wasOvershot
              ? 0.1
                  * (currentToTarget.getX() * currentToTarget.getX()
                      + currentToTarget.getY() * currentToTarget.getY())
              : 0.0;
      boolean lookaheadOvershot = dot <= exitThreshold;
      wasOvershot = lookaheadOvershot;

      if (lookaheadOvershot) {
        this.lookaheadTranslation = null;
        lookaheadDistanceMeters = currentDistanceMeters;
      } else {
        this.lookaheadTranslation = lookaheadTranslation;
      }

      Logger.recordOutput(
          logKey + "LookaheadPose",
          new Pose2d(
              lookaheadOvershot ? shooterTranslation : lookaheadTranslation,
              robotPose.get().getRotation()));
      Logger.recordOutput(
          logKey + "LookaheadDistanceFeet", Units.metersToFeet(lookaheadDistanceMeters));
      Logger.recordOutput(
          logKey + "CurrentDistanceFeet", Units.metersToFeet(currentDistanceMeters));
      Logger.recordOutput(
          logKey + "LookaheadCorrectionFeet",
          Units.metersToFeet(lookaheadDistanceMeters - currentDistanceMeters));
      Logger.recordOutput(logKey + "LookaheadOvershot", lookaheadOvershot);
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

  /**
   * Raw interpolated time-of-flight at {@code distanceMeters} with no lookahead applied. Used
   * externally to compute geometry-based drive speed limits.
   */
  public Optional<Time> getNaiveTOF(Distance distance) {
    OptionalDouble raw = interpolator.getTOF(distance.in(Meters));
    return raw.isPresent() ? Optional.of(Seconds.of(raw.getAsDouble())) : Optional.empty();
  }
}
