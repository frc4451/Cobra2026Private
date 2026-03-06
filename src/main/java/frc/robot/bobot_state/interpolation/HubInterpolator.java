package frc.robot.bobot_state.interpolation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import frc.robot.bobot_state.BobotState;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.FieldConstants;
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
        // Start of entries
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(7))
            .primaryVelocity(RotationsPerSecond.of(47))
            .pitch(interpolatorDefaultPitch)
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(8))
            .primaryVelocity(RotationsPerSecond.of(48))
            .pitch(interpolatorDefaultPitch)
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(9))
            .primaryVelocity(RotationsPerSecond.of(50))
            .pitch(interpolatorDefaultPitch)
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(10))
            .primaryVelocity(RotationsPerSecond.of(51))
            .pitch(interpolatorDefaultPitch)
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(11))
            .primaryVelocity(RotationsPerSecond.of(53))
            .pitch(interpolatorDefaultPitch)
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(12))
            .primaryVelocity(RotationsPerSecond.of(55))
            .pitch(interpolatorDefaultPitch)
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(13))
            .primaryVelocity(RotationsPerSecond.of(56.5))
            .pitch(interpolatorDefaultPitch)
            .build(),
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.of(17))
            .primaryVelocity(RotationsPerSecond.of(63.5))
            .pitch(interpolatorDefaultPitch)
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
        BobotState.getHubVARC().getTargetPose(),
        BobotState::getGlobalPose,
        interpolator);
  }
}
