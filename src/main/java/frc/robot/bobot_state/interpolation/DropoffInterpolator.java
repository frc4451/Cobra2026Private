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

public class DropoffInterpolator extends TargetInterpolator {
  /** NOTE fill this in with the actual maximum angle of the shooter */
  private static final ShootingInterpolator interpolator =
      new ShootingInterpolator(Degrees.of(28.0));

  private static final Angle interpolatorDefaultPitch = Degrees.of(28.0);

  /** NOTE fill this in with the actual distance to angle to velocity calculations */
  static {
    interpolator.addEntries(
        // Start of entries
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(13))
            .primaryVelocity(RotationsPerSecond.of(48))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.33))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(14))
            .primaryVelocity(RotationsPerSecond.of(52))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.733))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(15))
            .primaryVelocity(RotationsPerSecond.of(54))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.33))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(16))
            .primaryVelocity(RotationsPerSecond.of(56))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.33))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(17))
            .primaryVelocity(RotationsPerSecond.of(58))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.33))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(18))
            .primaryVelocity(RotationsPerSecond.of(60))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.34))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(19))
            .primaryVelocity(RotationsPerSecond.of(61.5))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.37))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(20))
            .primaryVelocity(RotationsPerSecond.of(63))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.39))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(21))
            .primaryVelocity(RotationsPerSecond.of(64.75))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.44))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(22))
            .primaryVelocity(RotationsPerSecond.of(67))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.50))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(23))
            .primaryVelocity(RotationsPerSecond.of(70))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.55))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(24))
            .primaryVelocity(RotationsPerSecond.of(74))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.60))
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(25))
            .primaryVelocity(RotationsPerSecond.of(78))
            .pitch(interpolatorDefaultPitch)
            .timeOfFlight(Seconds.of(1.76))
            .build());
    // DistanceAngleSpeedEntry.builder()
    //     .distanceToTarget(Feet.of(47))
    //     .primaryVelocity(RotationsPerSecond.of(92.4))
    //     .pitch(interpolatorDefaultPitch)
    //     .build());
  }

  public DropoffInterpolator() {
    super(
        TargetInterpolatorConfig.builder()
            .logKey("BobotState/Interpolation/Dropoff")
            .targetPose(() -> BobotState.getHubVARC().getTargetPose())
            .robotPose(() -> BobotState.getGlobalPose())
            .robotVelocity(() -> BobotState.getRobotVelocity())
            .lookaheadIterations(20)
            .interpolator(interpolator)
            .phaseDelay(Seconds.zero())
            .build());
  }
}
