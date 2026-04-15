package frc.robot.bobot_state.varc;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.bobot_state.BobotState;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.littletonrobotics.LoggedTunableNumber;
import org.robotzgarage.frc2026.math.varc.TargetVARC;

public class FuelClusterVARC extends TargetVARC {
  private static final LoggedTunableNumber clusteringThresholdDegrees =
      new LoggedTunableNumber("VARC/FuelCluster/ClusteringThresholdDegrees", 5.0);

  @Override
  public void update() {
    List<Pose3d> fuelPoses = BobotState.getKnownFuelPoses();

    if (fuelPoses.isEmpty()) {
      return;
    }

    Pose2d robotPose = BobotState.getGlobalPose();
    Translation2d robot = robotPose.getTranslation();

    // Sort all tracked fuel poses by their field-relative bearing from the robot
    List<Pose3d> sorted =
        fuelPoses.stream()
            .sorted(
                Comparator.comparingDouble(
                    pose ->
                        Math.toDegrees(
                            Math.atan2(pose.getY() - robot.getY(), pose.getX() - robot.getX()))))
            .toList();

    // Adjacent-compare clustering on bearing gap — mirrors the original yaw-based approach
    List<List<Pose3d>> clusters = new ArrayList<>();
    List<Pose3d> currentCluster = new ArrayList<>();
    currentCluster.add(sorted.get(0));

    for (int i = 1; i < sorted.size(); i++) {
      double prevBearing =
          Math.toDegrees(
              Math.atan2(
                  sorted.get(i - 1).getY() - robot.getY(),
                  sorted.get(i - 1).getX() - robot.getX()));
      double currBearing =
          Math.toDegrees(
              Math.atan2(sorted.get(i).getY() - robot.getY(), sorted.get(i).getX() - robot.getX()));

      if (Math.abs(currBearing - prevBearing) > clusteringThresholdDegrees.get()) {
        clusters.add(currentCluster);
        currentCluster = new ArrayList<>();
      }

      currentCluster.add(sorted.get(i));
    }
    clusters.add(currentCluster);

    // Largest cluster first
    clusters.sort((a, b) -> Integer.compare(b.size(), a.size()));
    List<Pose3d> largestCluster = clusters.get(0);

    // Centroid of the largest cluster
    double centroidX = largestCluster.stream().mapToDouble(Pose3d::getX).average().getAsDouble();
    double centroidY = largestCluster.stream().mapToDouble(Pose3d::getY).average().getAsDouble();
    Translation2d centroid = new Translation2d(centroidX, centroidY);

    // Rotation2d(dx, dy) computes atan2(dy, dx) — field-relative bearing to the cluster centroid,
    // centered to the intake
    this.rotationTarget =
        new Rotation2d(centroidX - robot.getX(), centroidY - robot.getY()).plus(Rotation2d.k180deg);
    this.distance = Meters.of(robot.getDistance(centroid));
    this.targetPose = new Pose2d(centroid, this.rotationTarget);
  }
}
