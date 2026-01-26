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
  private static final ShootingInterpolator interpolator = new ShootingInterpolator(Degrees.of(90));

  /** NOTE fill this in with the actual distance to angle to velocity calculations */
  static {
    interpolator.addEntries(
        DistanceAngleSpeedEntry.builder()
            .distanceToTarget(Feet.zero())
            .pitch(Degrees.zero())
            .primaryVelocity(RotationsPerSecond.zero())
            .build());
  }

  public HubInterpolator(Angle maximumAngle) {
    super(new Pose2d(), interpolator);
  }
}
