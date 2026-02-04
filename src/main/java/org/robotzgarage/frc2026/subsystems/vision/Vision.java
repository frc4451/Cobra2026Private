package org.robotzgarage.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.AprilTagCameraConfig;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.PoseEstimationMethod;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.PoseObservation;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.VisionSource;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.VisionSubsystemConfig;
import org.robotzgarage.frc2026.util.VirtualSubsystem;

public class Vision extends VirtualSubsystem {
  public static record AprilTagCamera(
      AprilTagIO io,
      AprilTagIOInputsAutoLogged inputs,
      VisionSource source,
      Alert disconnectedAlert) {}

  private final List<AprilTagCamera> aprilTagCameras = new ArrayList<>();

  private final String aprilTagLogRoot = "AprilTagCamera";
  private final String aggregateAprilTagLogRoot = "AggregateAprilTagCameras";

  private final Supplier<Pose2d> globalPoseSupplier;
  private final Consumer<PoseObservation> offerGlobalPoseObservation;
  private final Consumer<PoseObservation> offerConstrainedPoseObservation;

  public Vision(VisionSubsystemConfig config) {
    for (AprilTagCameraConfig aprilTagCameraConfig : config.aprilTagCameraConfigs()) {
      AprilTagIO io;

      switch (Constants.currentMode) {
        case REAL:
          io =
              new AprilTagIOPhoton(
                  aprilTagCameraConfig.source(),
                  config.aprilTagFieldLayout(),
                  config.robotHeadingSupplier(),
                  config.calculateConstrainedSolvePNP());
          break;
        case SIM:
          io =
              new AprilTagIOPhotonSim(
                  aprilTagCameraConfig.source(),
                  config.aprilTagFieldLayout(),
                  config.robotHeadingSupplier(),
                  config.calculateConstrainedSolvePNP(),
                  aprilTagCameraConfig.simConfig());
          break;
        case REPLAY:
        default:
          io = new AprilTagIO() {};
          break;
      }

      Alert disconnectedAlert =
          new Alert(
              aprilTagLogRoot + " " + aprilTagCameraConfig.source().name() + " is disconnected!",
              AlertType.kWarning);

      aprilTagCameras.add(
          new AprilTagCamera(
              io,
              new AprilTagIOInputsAutoLogged(),
              aprilTagCameraConfig.source(),
              disconnectedAlert));
    }

    this.globalPoseSupplier = config.globalPoseSupplier();
    this.offerConstrainedPoseObservation = config.offerConstrainedPoseObservation();
    this.offerGlobalPoseObservation = config.offerGlobalPoseObservation();
  }

  @Override
  public void periodic() {
    List<Translation2d> validCorners = new ArrayList<>();
    List<Translation2d> rejectedCorners = new ArrayList<>();

    List<Integer> validIds = new ArrayList<>();
    List<Integer> rejectedIds = new ArrayList<>();

    List<PoseObservation> validPoseObservations = new ArrayList<>();
    List<PoseObservation> rejectedPoseObservations = new ArrayList<>();

    List<Pose3d> validPoses = new ArrayList<>();
    List<Pose3d> rejectedPoses = new ArrayList<>();

    List<Pose3d> validAprilTagPoses = new ArrayList<>();
    List<Pose3d> rejectedAprilTagPoses = new ArrayList<>();

    for (AprilTagCamera cam : aprilTagCameras) {
      cam.io.updateInputs(cam.inputs);
      Logger.processInputs(aprilTagLogRoot + "/" + cam.source.name(), cam.inputs);

      cam.disconnectedAlert.set(!cam.inputs.connected);

      validCorners.addAll(Arrays.asList(cam.inputs.validCorners));
      rejectedCorners.addAll(Arrays.asList(cam.inputs.rejectedCorners));

      validIds.addAll(Arrays.stream(cam.inputs.validIds).boxed().toList());
      rejectedIds.addAll(Arrays.stream(cam.inputs.rejectedIds).boxed().toList());

      validPoseObservations.addAll(Arrays.asList(cam.inputs.validPoseObservations));
      rejectedPoseObservations.addAll(Arrays.asList(cam.inputs.rejectedPoseObservations));

      validPoses.addAll(Arrays.asList(cam.inputs.validPoses));
      rejectedPoses.addAll(Arrays.asList(cam.inputs.rejectedPoses));

      validAprilTagPoses.addAll(Arrays.asList(cam.inputs.validAprilTagPoses));
      rejectedAprilTagPoses.addAll(Arrays.asList(cam.inputs.rejectedAprilTagPoses));

      for (PoseObservation observation : cam.inputs.validPoseObservations) {
        if ((observation.method() == PoseEstimationMethod.TRIG)
            || (observation.method() == PoseEstimationMethod.CONSTRAINED)) {
          this.offerConstrainedPoseObservation.accept(observation);
        } else {
          this.offerGlobalPoseObservation.accept(observation);
        }
      }
    }

    Logger.recordOutput(
        aggregateAprilTagLogRoot + "/ValidCorners", validCorners.toArray(Translation2d[]::new));
    Logger.recordOutput(
        aggregateAprilTagLogRoot + "/RejectedCorners",
        rejectedCorners.toArray(Translation2d[]::new));

    Logger.recordOutput(
        aggregateAprilTagLogRoot + "/ValidIds",
        validIds.stream().mapToInt(Integer::intValue).toArray());
    Logger.recordOutput(
        aggregateAprilTagLogRoot + "/RejectedIds",
        rejectedIds.stream().mapToInt(Integer::intValue).toArray());

    Logger.recordOutput(
        aggregateAprilTagLogRoot + "/ValidPoseObservations",
        validPoseObservations.toArray(PoseObservation[]::new));
    Logger.recordOutput(
        aggregateAprilTagLogRoot + "/RejectedPoseObservations",
        rejectedPoseObservations.toArray(PoseObservation[]::new));

    Logger.recordOutput(
        aggregateAprilTagLogRoot + "/ValidPoses", validPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        aggregateAprilTagLogRoot + "/RejectedPoses", rejectedPoses.toArray(Pose3d[]::new));

    Logger.recordOutput(
        aggregateAprilTagLogRoot + "/ValidAprilTagPoses",
        validAprilTagPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        aggregateAprilTagLogRoot + "/RejectedAprilTagPoses",
        rejectedAprilTagPoses.toArray(Pose3d[]::new));
  }

  @Override
  public void simulationPeriodic() {
    VisionConstants.aprilTagSim.ifPresent(
        aprilTagSim -> aprilTagSim.update(globalPoseSupplier.get()));
  }
}
