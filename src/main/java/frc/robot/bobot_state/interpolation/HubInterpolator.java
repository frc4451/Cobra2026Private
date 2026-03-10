package frc.robot.bobot_state.interpolation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import frc.robot.bobot_state.BobotState;
import org.robotzgarage.frc2026.math.interpolation.ShootingInterpolator;
import org.robotzgarage.frc2026.math.interpolation.ShootingInterpolator.DistanceAngleSpeedEntry;
import org.robotzgarage.frc2026.math.interpolation.TargetInterpolator;

public class HubInterpolator extends TargetInterpolator {
  /** NOTE fill this in with the actual maximum angle of the shooter */
  private static final ShootingInterpolator interpolator =
      new ShootingInterpolator(Degrees.of(28.0));

  private static final Angle interpolatorDefaultPitch = Degrees.of(28.0);

  /** NOTE fill this in with the actual distance to angle to velocity with tof calculations */
  static {
    interpolator.addEntries(
        // Start of entries
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(7))
            .primaryVelocity(RotationsPerSecond.of(47))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(0.64))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(8))
            .primaryVelocity(RotationsPerSecond.of(48))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(0.71))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(9))
            .primaryVelocity(RotationsPerSecond.of(50))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(0.74))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(10))
            .primaryVelocity(RotationsPerSecond.of(51))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(0.81))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(11))
            .primaryVelocity(RotationsPerSecond.of(53))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(0.96))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(12))
            .primaryVelocity(RotationsPerSecond.of(55))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.08))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(13))
            .primaryVelocity(RotationsPerSecond.of(56.5))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.33))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(14))
            .primaryVelocity(RotationsPerSecond.of(58.4))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.36))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(15))
            .primaryVelocity(RotationsPerSecond.of(60))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.37))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(17))
            .primaryVelocity(RotationsPerSecond.of(63.5))
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
            .lookaheadIterations(20)
            .interpolator(interpolator)
            .phaseDelay(Seconds.zero())
            .build());
  }
}
