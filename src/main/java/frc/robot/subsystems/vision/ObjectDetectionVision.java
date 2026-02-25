package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.bobot_state.BobotState;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.ObjectDetectionTrackedResult;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.VisionSource;
import org.robotzgarage.frc2026.util.VirtualSubsystem;

public class ObjectDetectionVision extends VirtualSubsystem {
  public static record ObjectDetectionCamera(
      ObjectDetectionIO io,
      ObjectDetectionIOInputsAutoLogged inputs,
      VisionSource source,
      Alert disconnectedAlert) {}

  private final ObjectDetectionCamera intakeFuelDetection;
  private final ObjectDetectionCamera hopperFuelDetection;

  private final String objectDetectionLogRoot = "ObjectDetection";

  private static final LoggedTunableNumber clusteringThresholdDegrees =
      new LoggedTunableNumber("ObjectDetection/Intake/ClusteringThresholdDegrees", 5.0);

  private final List<List<ObjectDetectionTrackedResult>> clusteredObjectDetectionResults =
      new ArrayList<>();

  @Getter private boolean intakeCanSeeFuel = false;
  @Getter private Rotation2d intakeRelativeYawToClosestCluster = Rotation2d.kZero;
  @Getter private Rotation2d intakeRelativePitchToClosestCluster = Rotation2d.kZero;
  @Getter private int intakeLargestClusterSize = 0;
  @Getter private Optional<Pose2d> estimatedCenterOfCluster = Optional.empty();

  public ObjectDetectionVision() {
    ObjectDetectionIO intakeFuelDetectionIO;
    ObjectDetectionIO hopperFuelDetectionIO;

    switch (Constants.currentMode) {
      case REAL:
        intakeFuelDetectionIO =
            new ObjectDetectionIOPhoton(
                VisionConstants.intakeFuelDetection.source(),
                () -> BobotState.getGlobalPose(),
                true);
        hopperFuelDetectionIO =
            new ObjectDetectionIOPhoton(
                VisionConstants.hopperFuelDetection.source(),
                () -> BobotState.getGlobalPose(),
                false);
        break;
      case SIM:
        intakeFuelDetectionIO =
            new ObjectDetectionIOPhotonSim(
                VisionConstants.intakeFuelDetection.source(),
                () -> BobotState.getGlobalPose(),
                true,
                VisionConstants.intakeFuelDetection.simCameraConfig());
        hopperFuelDetectionIO =
            new ObjectDetectionIOPhotonSim(
                VisionConstants.hopperFuelDetection.source(),
                () -> BobotState.getGlobalPose(),
                false,
                VisionConstants.hopperFuelDetection.simCameraConfig());
        break;
      case REPLAY:
      default:
        intakeFuelDetectionIO = new ObjectDetectionIO() {};
        hopperFuelDetectionIO = new ObjectDetectionIO() {};
        break;
    }

    intakeFuelDetection =
        new ObjectDetectionCamera(
            intakeFuelDetectionIO,
            new ObjectDetectionIOInputsAutoLogged(),
            VisionConstants.intakeFuelDetection.source(),
            new Alert(
                objectDetectionLogRoot
                    + " "
                    + VisionConstants.intakeFuelDetection.source().name()
                    + " is disconnected!",
                AlertType.kWarning));
    hopperFuelDetection =
        new ObjectDetectionCamera(
            hopperFuelDetectionIO,
            new ObjectDetectionIOInputsAutoLogged(),
            VisionConstants.hopperFuelDetection.source(),
            new Alert(
                objectDetectionLogRoot
                    + " "
                    + VisionConstants.hopperFuelDetection.source().name()
                    + " is disconnected!",
                AlertType.kWarning));
  }

  @Override
  public void periodic() {
    for (ObjectDetectionCamera cam : List.of(intakeFuelDetection, hopperFuelDetection)) {
      cam.io.updateInputs(cam.inputs);
      Logger.processInputs(objectDetectionLogRoot + "/" + cam.source.name(), cam.inputs);
      cam.disconnectedAlert.set(!cam.inputs.connected);
    }

    {
      // Vision updates for Intake Camera
      clusteredObjectDetectionResults.clear();

      // Collect all yaws and compile them to a list sorted by yaw
      List<ObjectDetectionTrackedResult> trackedResultsList =
          Arrays.asList(intakeFuelDetection.inputs.relativeTargets);

      trackedResultsList.sort(Comparator.comparingDouble(target -> target.yaw().getDegrees()));

      this.intakeCanSeeFuel = !trackedResultsList.isEmpty();

      if (this.intakeCanSeeFuel) {
        // Cluster our objects by YAW with respect to threshold. This is a "simple"
        // reverse-linked lookup comparison
        List<ObjectDetectionTrackedResult> currentCluster = new ArrayList<>();

        currentCluster.add(trackedResultsList.get(0));

        for (int i = 1; i < trackedResultsList.size(); ++i) {
          double previousYawDeg = trackedResultsList.get(i - 1).yaw().getDegrees();
          double currentYawDeg = trackedResultsList.get(i).yaw().getDegrees();

          if (Math.abs(currentYawDeg - previousYawDeg) > clusteringThresholdDegrees.get()) {
            clusteredObjectDetectionResults.add(currentCluster);
            currentCluster = new ArrayList<>();
          }

          currentCluster.add(trackedResultsList.get(i));
        }
        clusteredObjectDetectionResults.add(currentCluster);

        clusteredObjectDetectionResults.sort((a, b) -> Integer.compare(b.size(), a.size()));

        List<ObjectDetectionTrackedResult> largestCluster = clusteredObjectDetectionResults.get(0);

        double totalArea =
            largestCluster.stream().mapToDouble(ObjectDetectionTrackedResult::area).sum();

        double avgYawDeg =
            largestCluster.stream().mapToDouble(t -> t.yaw().getDegrees()).average().getAsDouble();

        double avgPitchDeg =
            largestCluster.stream()
                .mapToDouble(t -> t.pitch().getDegrees())
                .average()
                .getAsDouble();

        double distanceFromCenterOfClusterMeters =
            PhotonUtils.calculateDistanceToTargetMeters(
                intakeFuelDetection.source().robotToCamera().upPositive().in(Meters),
                VisionConstants.fuelHeightMeters,
                -intakeFuelDetection.source().robotToCamera().pitchCCWPositive().in(Radians),
                Units.degreesToRadians(avgPitchDeg));

        Translation2d estimatedCenterOfCluster =
            PhotonUtils.estimateCameraToTargetTranslation(
                distanceFromCenterOfClusterMeters, Rotation2d.fromDegrees(avgYawDeg));

        this.intakeRelativePitchToClosestCluster = Rotation2d.fromDegrees(avgPitchDeg);
        this.intakeRelativeYawToClosestCluster = Rotation2d.fromDegrees(avgYawDeg);

        this.intakeLargestClusterSize =
            clusteredObjectDetectionResults.isEmpty()
                ? 0
                : clusteredObjectDetectionResults.get(0).size();
        this.estimatedCenterOfCluster =
            Optional.of(new Pose2d(estimatedCenterOfCluster, Rotation2d.kZero));
      } else {
        this.intakeLargestClusterSize = 0;
        this.estimatedCenterOfCluster = Optional.empty();
      }

      Logger.recordOutput(objectDetectionLogRoot + "/Intake/CanSeeFuel", this.intakeCanSeeFuel);
      Logger.recordOutput(
          objectDetectionLogRoot + "/Intake/AverageYawDegrees",
          this.intakeRelativePitchToClosestCluster.getDegrees());
      Logger.recordOutput(
          objectDetectionLogRoot + "/Intake/AveragePitchDegrees",
          this.intakeRelativeYawToClosestCluster.getDegrees());
      Logger.recordOutput(
          objectDetectionLogRoot + "/Intake/LargestClusterSize/", this.intakeLargestClusterSize);
      for (int i = 0; i < clusteredObjectDetectionResults.size(); ++i) {
        Logger.recordOutput(
            objectDetectionLogRoot + "/Intake/ClusteredResults/" + i,
            clusteredObjectDetectionResults.get(i).toArray(ObjectDetectionTrackedResult[]::new));
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    VisionConstants.objectDetectionSim.ifPresent(
        objectDetectionSim -> objectDetectionSim.update(BobotState.getGlobalPose()));
  }

  @AutoLogOutput(key = "ObjectDetection/Hopper/NumFuelCollected")
  public int getNumFuelInHopperView() {
    return hopperFuelDetection.inputs.numObjectsInView;
  }

  @AutoLogOutput(key = "ObjectDetection/Intake/NumFuelInView")
  public int getNumFuelInIntakeView() {
    return intakeFuelDetection.inputs.numObjectsInView;
  }
}
