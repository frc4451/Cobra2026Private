package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.bobot_state.BobotState;
import java.util.List;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.ObjectDetectionStrategy;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.VisionSource;
import org.robotzgarage.frc2026.util.VirtualSubsystem;

public class ObjectDetectionVision extends VirtualSubsystem {
  public static record ObjectDetectionCamera(
      ObjectDetectionIO io,
      ObjectDetectionIOInputsAutoLogged inputs,
      VisionSource source,
      Alert disconnectedAlert) {}

  private final ObjectDetectionCamera intakeFuelDetection;
  // private final ObjectDetectionCamera hopperFuelDetection;

  private final String objectDetectionLogRoot = "ObjectDetection";

  private final FuelTracker fuelTracker = new FuelTracker();

  /** True only when the intake camera has fuel in frame this loop. */
  @Getter private boolean intakeCanSeeFuel = false;

  /**
   * True when any fuel is held in the tracker's memory window (up to {@code trackingMemorySeconds}
   * stale).
   */
  @Getter private boolean fuelIsTracked = false;

  @Getter private Rotation2d intakeRelativeYawToClosestCluster = Rotation2d.kZero;
  @Getter private Rotation2d intakeRelativePitchToClosestCluster = Rotation2d.kZero;
  @Getter private int intakeLargestClusterSize = 0;
  @Getter private Optional<Pose2d> estimatedCenterOfCluster = Optional.empty();

  public ObjectDetectionVision() {
    ObjectDetectionIO intakeFuelDetectionIO;
    // ObjectDetectionIO hopperFuelDetectionIO;

    switch (Constants.currentMode) {
      case REAL:
        intakeFuelDetectionIO =
            new ObjectDetectionIOPhoton(
                VisionConstants.intakeFuelDetection.source(),
                () -> BobotState.getGlobalPose(),
                VisionConstants.intakeFuelDetection.objectDetectionStrategies());
        // hopperFuelDetectionIO =
        //     new ObjectDetectionIOPhoton(
        //         VisionConstants.hopperFuelDetection.source(),
        //         () -> BobotState.getGlobalPose(),
        //         VisionConstants.hopperFuelDetection.objectDetectionStrategies());
        break;
      case SIM:
        intakeFuelDetectionIO =
            new ObjectDetectionIOPhotonSim(
                VisionConstants.intakeFuelDetection.source(),
                () -> BobotState.getGlobalPose(),
                VisionConstants.intakeFuelDetection.objectDetectionStrategies(),
                VisionConstants.intakeFuelDetection.simCameraConfig());
        // hopperFuelDetectionIO =
        //     new ObjectDetectionIOPhotonSim(
        //         VisionConstants.hopperFuelDetection.source(),
        //         () -> BobotState.getGlobalPose(),
        //         VisionConstants.hopperFuelDetection.objectDetectionStrategies(),
        //         VisionConstants.hopperFuelDetection.simCameraConfig());
        break;
      case REPLAY:
      default:
        intakeFuelDetectionIO = new ObjectDetectionIO() {};
        // hopperFuelDetectionIO = new ObjectDetectionIO() {};
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
    // hopperFuelDetection =
    //     new ObjectDetectionCamera(
    //         hopperFuelDetectionIO,
    //         new ObjectDetectionIOInputsAutoLogged(),
    //         VisionConstants.hopperFuelDetection.source(),
    //         new Alert(
    //             objectDetectionLogRoot
    //                 + " "
    //                 + VisionConstants.hopperFuelDetection.source().name()
    //                 + " is disconnected!",
    //             AlertType.kWarning));
  }

  @Override
  public void periodic() {
    for (ObjectDetectionCamera cam : List.of(intakeFuelDetection)) {
      // Process Inputs
      cam.io.updateInputs(cam.inputs);
      Logger.processInputs(objectDetectionLogRoot + "/" + cam.source.name(), cam.inputs);
      cam.disconnectedAlert.set(!cam.inputs.connected);

      // Check if this camera is supposed to track poses for Fuel, then add to FuelTracker
      if (cam.io.getStrategies().contains(ObjectDetectionStrategy.POSE_TRACKING)) {
        fuelTracker.addMoreTrackedPoses(List.of(cam.inputs.validObjectPoses));
      }
    }

    // Clustering

    fuelTracker.update();
    List<Pose3d> knownFuelPoses = fuelTracker.getObjectPoses();
    intakeCanSeeFuel = intakeFuelDetection.inputs.numObjectsInView > 0;
    fuelIsTracked = !knownFuelPoses.isEmpty();

    Logger.recordOutput(objectDetectionLogRoot + "/IntakeCanSeeFuel", intakeCanSeeFuel);
    Logger.recordOutput(objectDetectionLogRoot + "/FuelIsTracked", fuelIsTracked);
    Logger.recordOutput(
        objectDetectionLogRoot + "/KnownFuelPoses", knownFuelPoses.toArray(Pose3d[]::new));

    // Send results to BobotState
    BobotState.setKnownFuelPoses(knownFuelPoses);
  }

  @Override
  public void simulationPeriodic() {
    VisionConstants.objectDetectionSim.ifPresent(
        objectDetectionSim -> objectDetectionSim.update(BobotState.getGlobalPose()));
  }

  // @AutoLogOutput(key = "ObjectDetection/Hopper/NumFuelCollected")
  // public int getNumFuelInHopperView() {
  //   return hopperFuelDetection.inputs.numObjectsInView;
  // }

  @AutoLogOutput(key = "ObjectDetection/Intake/NumFuelInView")
  public int getNumFuelInIntakeView() {
    return intakeFuelDetection.inputs.numObjectsInView;
  }
}
