package org.robotzgarage.frc2026.math.interpolation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import java.util.OptionalDouble;
import lombok.Builder;
import org.robotzgarage.frc2026.util.FieldUtils;

public class ShootingInterpolator {
  private final Angle maximumAngle;
  private final Angle blueCloseAngleFudgeFactor;
  private final Angle blueFarAngleFudgeFactor;
  private final Angle redCloseAngleFudgeFactor;
  private final Angle redFarAngleFudgeFactor;

  /**
   * Given an angle for the mechanism, assign velocities for the "left" and "right" motor
   * controllers.
   */
  @Builder
  public static record InterpolatedCalculation(
      Angle angle,
      AngularVelocity primaryVelocity,
      AngularVelocity secondaryVelocity,
      Time timeOfFlight) {}

  /**
   * Given a distance to target, provide a pitch for your shooter in addition to the velocities
   * needed for the "left" and "right" motors
   *
   * <p>Values are internally calculated to meters, degrees, and rotations-per-second
   */
  @Builder
  public static record DistanceAngleSpeedEntry(
      Distance distanceToTarget,
      Angle pitch,
      AngularVelocity primaryVelocity,
      AngularVelocity secondaryVelocity,
      Time timeOfFlight) {

    public DistanceAngleSpeedEntry {      
      if (secondaryVelocity == null) {
        secondaryVelocity = primaryVelocity;
      }
    }
  }

  /**
   * Interpolation table manager for distance-to-targets tailored for shooter mechanisms.
   *
   * @param maximumAngle maximum angle allowed for the mechanism
   */
  public ShootingInterpolator(Angle maximumAngle) {
    this(maximumAngle, Degrees.zero(), Degrees.zero(), Degrees.zero(), Degrees.zero());
  }

  /**
   * Interpolation table manager for distance-to-targets
   *
   * @param maximumAngle maximum angle allowed by the mechanism
   * @param blueCloseAngleFudgeFactor fudge factor for close-up blue alliance
   * @param blueFarAngleFudgeFactor fudge factor for far-away blue alliance
   * @param redCloseAngleFudgeFactor fudge factor for close-up red alliance
   * @param redFarAngleFudgeFactor fudge factor for far-away red alliance
   */
  public ShootingInterpolator(
      Angle maximumAngle,
      Angle blueCloseAngleFudgeFactor,
      Angle blueFarAngleFudgeFactor,
      Angle redCloseAngleFudgeFactor,
      Angle redFarAngleFudgeFactor) {
    this.maximumAngle = maximumAngle;
    this.blueCloseAngleFudgeFactor = blueCloseAngleFudgeFactor;
    this.blueFarAngleFudgeFactor = blueFarAngleFudgeFactor;
    this.redCloseAngleFudgeFactor = redCloseAngleFudgeFactor;
    this.redFarAngleFudgeFactor = redFarAngleFudgeFactor;
  }

  private final InterpolatingDoubleTreeMap blueDistanceAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap redDistanceAngleMap = new InterpolatingDoubleTreeMap();

  private final InterpolatingDoubleTreeMap distanceToVelocityLeftMap =
      new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap distanceToVelocityRightMap =
      new InterpolatingDoubleTreeMap();

  private final InterpolatingDoubleTreeMap distanceToTOFMap = new InterpolatingDoubleTreeMap();
  private boolean hasTOFData = false;

  public void addEntries(DistanceAngleSpeedEntry... entries) {
    for (DistanceAngleSpeedEntry entry : entries) {
      addEntry(entry);
    }
  }

  public void addEntry(DistanceAngleSpeedEntry entry) {
    boolean isClose = entry.distanceToTarget.lt(Meters.of(12));

    Angle blueFudgeFactor = isClose ? blueCloseAngleFudgeFactor : blueFarAngleFudgeFactor;

    Angle redFudgeFactor = isClose ? redCloseAngleFudgeFactor : redFarAngleFudgeFactor;

    blueDistanceAngleMap.put(
        entry.distanceToTarget.in(Meters),
        Math.min(entry.pitch.plus(blueFudgeFactor).in(Degrees), maximumAngle.in(Degrees)));

    redDistanceAngleMap.put(
        entry.distanceToTarget.in(Meters),
        Math.min(entry.pitch.plus(redFudgeFactor).in(Degrees), maximumAngle.in(Degrees)));

    distanceToVelocityLeftMap.put(
        entry.distanceToTarget.in(Meters), entry.primaryVelocity.in(RotationsPerSecond));

    distanceToVelocityRightMap.put(
        entry.distanceToTarget.in(Meters), entry.secondaryVelocity.in(RotationsPerSecond));

    if (entry.timeOfFlight() != null) {
      distanceToTOFMap.put(entry.distanceToTarget.in(Meters), entry.timeOfFlight().in(Seconds));
      hasTOFData = true;
    }
  }

  public OptionalDouble getTOF(double distanceMeters) {
    if (!hasTOFData) return OptionalDouble.empty();
    return OptionalDouble.of(distanceToTOFMap.get(distanceMeters));
  }

  public InterpolatedCalculation calculateInterpolation(Distance distance) {
    InterpolatingDoubleTreeMap angleMap =
        FieldUtils.isBlueAlliance() ? blueDistanceAngleMap : redDistanceAngleMap;

    double distanceMeters = distance.in(Meters);

    Angle angleCalculation = Degrees.of(angleMap.get(distanceMeters));
    AngularVelocity leftSpeedCalculation =
        RotationsPerSecond.of(distanceToVelocityLeftMap.get(distanceMeters));
    AngularVelocity rightSpeedCalculation =
        RotationsPerSecond.of(distanceToVelocityRightMap.get(distanceMeters));

    Time tof = hasTOFData ? Seconds.of(distanceToTOFMap.get(distanceMeters)) : null;
    return new InterpolatedCalculation(
        angleCalculation, leftSpeedCalculation, rightSpeedCalculation, tof);
  }
}
