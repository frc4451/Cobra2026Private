package frc.robot.bobot_state.interpolation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import org.robotzgarage.frc2026.math.interpolation.ShootingInterpolator;
import org.robotzgarage.frc2026.math.interpolation.ShootingInterpolator.DistanceAngleSpeedEntry;
import org.robotzgarage.frc2026.math.interpolation.TargetInterpolator;

public class HubInterpolator extends TargetInterpolator {
  /** NOTE fill this in with the actual maximum angle of the shooter */
  private static final ShootingInterpolator interpolator =
      new ShootingInterpolator(Degrees.of(28.0));

  private static final Angle interpolatorDefaultPitch = Degrees.of(28.0);

  /** NOTE fill this in with the actual distance to angle to velocity calculations */
  static {
    interpolator.addEntries(
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(4))
            .primaryVelocity(RotationsPerSecond.of(63))
            .pitch(interpolatorDefaultPitch)
            .build());
    DistanceAngleSpeedEntry.builder()
        .distanceToTarget(Feet.of(6))
        .primaryVelocity(RotationsPerSecond.of(69))
        .pitch(interpolatorDefaultPitch)
        .build();
    DistanceAngleSpeedEntry.builder()
        .distanceToTarget(Feet.of(8))
        .primaryVelocity(RotationsPerSecond.of(75))
        .pitch(interpolatorDefaultPitch)
        .build();
    DistanceAngleSpeedEntry.builder()
        .distanceToTarget(Feet.of(10))
        .primaryVelocity(RotationsPerSecond.of(79))
        .pitch(interpolatorDefaultPitch)
        .build();
    DistanceAngleSpeedEntry.builder()
        .distanceToTarget(Feet.of(12))
        .primaryVelocity(RotationsPerSecond.of(85))
        .pitch(interpolatorDefaultPitch)
        .build();
  }

  public HubInterpolator() {
    super(new Pose2d(), interpolator);
  }
}
