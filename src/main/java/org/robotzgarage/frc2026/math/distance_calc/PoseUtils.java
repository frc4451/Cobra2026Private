package org.robotzgarage.frc2026.math.distance_calc;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

// We kept these because we could not part with them, it was too sad
// (and post-hoc found the justification that this makes less objects)
public class PoseUtils {
  public static Pose2d getParallelOffsetPose(Pose2d pose, Distance offset) {
    Translation2d offsetTranslation =
        pose.getTranslation()
            .plus(
                new Translation2d(
                    // Add 90 degrees to all trig functions
                    // so it is offset parallel to the face of the tag
                    -offset.in(Meters) * pose.getRotation().getSin(),
                    offset.in(Meters) * pose.getRotation().getCos()));

    return new Pose2d(offsetTranslation, pose.getRotation());

    // But this way simpler to reason about & understand
    // return pose.transformBy(new Transform2d(0, offsetMeters, Rotation2d.kZero));
  }

  public static Pose2d getPerpendicularOffsetPose(Pose2d pose, Distance perpendicularOffset) {
    Translation2d offsetTranslation =
        pose.getTranslation()
            .plus(
                new Translation2d(
                    perpendicularOffset.in(Meters) * pose.getRotation().getCos(),
                    perpendicularOffset.in(Meters) * pose.getRotation().getSin()));

    return new Pose2d(offsetTranslation, pose.getRotation());

    // But this way simpler to reason about & understand
    // return pose.transformBy(new Transform2d(perpendicularOffsetMeters, 0, Rotation2d.kZero));
  }

  public static Pose2d getOffsetPose(
      Pose2d pose, Distance parellelOffset, Distance perpendicularOffset) {
    return getPerpendicularOffsetPose(
        getParallelOffsetPose(pose, parellelOffset), perpendicularOffset);
  }

  /**
   * @see https://en.wikipedia.org/wiki/Vector_projection#Scalar_projection
   */
  public static double getParallelError(Pose2d origin, Pose2d target) {
    Translation2d originToTarget = origin.minus(target).getTranslation();
    Rotation2d angleBetween = originToTarget.getAngle();
    double parallelError = originToTarget.getNorm() * angleBetween.getSin();

    return parallelError;

    // return origin.minus(target).getY();
  }

  /**
   * @see https://en.wikipedia.org/wiki/Vector_projection#Scalar_projection
   */
  public static double getPerpendicularError(Pose2d origin, Pose2d target) {
    Translation2d originToTarget = origin.minus(target).getTranslation();
    Rotation2d angleBetween = originToTarget.getAngle();
    double perpendicularError = originToTarget.getNorm() * angleBetween.getCos();

    return perpendicularError;

    // return -origin.minus(target).getX();
  }

  public static Pose2d plusRotation(Pose2d pose, Rotation2d rotation) {
    return new Pose2d(pose.getTranslation(), pose.getRotation().plus(rotation));
  }

  public static Pose2d plus(Pose2d a, Pose2d b) {
    return new Pose2d(
        a.getTranslation().plus(b.getTranslation()), a.getRotation().plus(b.getRotation()));
  }

  public static Pose2d withRotation(Pose2d pose, Rotation2d rotation) {
    return new Pose2d(pose.getTranslation(), rotation);
  }

  /**
   * Given a "current" pose and "target" pose, return the hypotenuse between them
   *
   * @param current "current" or where the robot likely is
   * @param target "target" or where the robot is interested
   * @return hypotenuse distance between poses
   */
  public static Distance getDistanceBetweenPoses(Pose2d current, Pose2d target) {
    Translation2d currentTranslation = current.getTranslation();
    Translation2d targetTranslation = target.getTranslation();
    double distanceToTarget = currentTranslation.getDistance(targetTranslation);
    return Meters.of(distanceToTarget);
  }

  public static Transform2d transform3dTo2d(Transform3d other) {
    return new Transform2d(
        other.getTranslation().toTranslation2d(), other.getRotation().toRotation2d());
  }

  public static Translation2d averageOfProvidedTranslations(Translation2d... translations) {
    Translation2d average = Translation2d.kZero;

    for (Translation2d pose : translations) {
      average = average.plus(pose);
    }
    return average.div(translations.length);
  }

  public static Translation3d averageOfProvidedTranslations(Translation3d... translations) {
    Translation3d average = Translation3d.kZero;

    for (Translation3d pose : translations) {
      average = average.plus(pose);
    }
    return average.div(translations.length);
  }

  public static Rotation2d getClosestAngle(Rotation2d current, Rotation2d... targets) {
    Rotation2d closest = Rotation2d.fromDegrees(Double.MAX_VALUE);
    Rotation2d result = Rotation2d.fromDegrees(Double.MAX_VALUE);

    for (Rotation2d target : targets) {
      if (Math.abs(target.minus(current).getDegrees()) < closest.getDegrees()) {
        closest = Rotation2d.fromDegrees(Math.abs(target.minus(current).getDegrees()));
        result = target;
      }
    }

    return result;
  }
}
