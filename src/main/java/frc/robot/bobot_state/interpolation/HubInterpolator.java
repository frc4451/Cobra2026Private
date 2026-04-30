package frc.robot.bobot_state.interpolation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import frc.robot.bobot_state.BobotState;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.robotzgarage.frc2026.math.interpolation.ShootingInterpolator;
import org.robotzgarage.frc2026.math.interpolation.ShootingInterpolator.DistanceAngleSpeedEntry;
import org.robotzgarage.frc2026.math.interpolation.TargetInterpolator;

public class HubInterpolator extends TargetInterpolator {
  /** NOTE fill this in with the actual maximum angle of the shooter */
  private static final ShootingInterpolator interpolator =
      new ShootingInterpolator(Degrees.of(28.0));

  private static final Angle interpolatorDefaultPitch = Degrees.of(28.0);

  /**
   * Distance-to-velocity interpolation table for hub shots.
   *
   * <p>Each entry maps a shot distance (feet) to a flywheel velocity (RPS). The {@code
   * timeOfFlight} value is the ball's <em>in-air</em> time only — from the moment the ball exits
   * the shooter until it enters the hub. It does NOT include indexer feed delay or flywheel spinup;
   * those belong in {@code phaseDelay} on the config.
   *
   * <p>The table must cover the full expected lookahead range: at max robot speed, the lookahead
   * distance can exceed the actual shot distance by up to {@code maxSpeed × maxTOF}. Entries
   * outside the table bounds will clamp to the nearest edge value.
   */
  static {
    interpolator.addEntries(
        // Start of entries
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(7))
            .primaryVelocity(RotationsPerSecond.of(43))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(0.64))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(8))
            .primaryVelocity(RotationsPerSecond.of(43))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(0.71))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(9))
            .primaryVelocity(RotationsPerSecond.of(45))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(0.74))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(10))
            .primaryVelocity(RotationsPerSecond.of(47))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(0.81))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(11))
            .primaryVelocity(RotationsPerSecond.of(47.5))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(0.96))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(12))
            .primaryVelocity(RotationsPerSecond.of(49))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.08))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(13))
            .primaryVelocity(RotationsPerSecond.of(51.5))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.33))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(14))
            .primaryVelocity(RotationsPerSecond.of(53))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.36))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(15))
            .primaryVelocity(RotationsPerSecond.of(53.5))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.37))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(16))
            .primaryVelocity(RotationsPerSecond.of(56))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.37))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(16.85))
            .primaryVelocity(RotationsPerSecond.of(57))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.37))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(17))
            .primaryVelocity(RotationsPerSecond.of(57.25))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.35))
            .build()
        // DistanceAngleSpeedEntry.builder()
        //     .distanceToTarget(Feet.of(6))
        //     .primaryVelocity(RotationsPerSecond.of(69))
        //     .pitch(interpolatorDefaultPitch)
        //     .build(),
        // DistanceAngleSpeedEntry.builder()
        //     .distanceToTarget(Feet.of(8))
        //     .primaryVelocity(RotationsPerSecond.of(75))
        //     .pitch(interpolatorDefaultPitch)
        //     .build(),
        // DistanceAngleSpeedEntry.builder()
        //     .distanceToTarget(Feet.of(10))
        //     .primaryVelocity(RotationsPerSecond.of(79))
        //     .pitch(interpolatorDefaultPitch)
        //     .build(),
        // DistanceAngleSpeedEntry.builder()
        //     .distanceToTarget(Feet.of(12))
        //     .primaryVelocity(RotationsPerSecond.of(85))
        //     .pitch(interpolatorDefaultPitch)
        //     .build()
        // End of entries
        );
  }

  public HubInterpolator() {
    super(
        TargetInterpolatorConfig.builder()
            .logKey("BobotState/Interpolation/Hub")
            .targetPose(() -> BobotState.getHubVARC().getTargetPose())
            .robotPose(() -> BobotState.getGlobalPose())
            .robotVelocity(() -> BobotState.getRobotVelocity())
            .robotToShooter(() -> ShooterConstants.robotToShooter)
            .lookaheadIterations(20)
            .interpolator(interpolator)
            .phaseDelay(Seconds.zero())
            .build());
  }
}
