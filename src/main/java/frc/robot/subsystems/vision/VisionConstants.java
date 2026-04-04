package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.bobot_state.BobotState;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.simulation.VisionSystemSim;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.AprilTagCameraConfig;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.ObjectDetectionCameraConfig;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.ObjectDetectionStrategy;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.PoseEstimationMethod;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.RobotRelativeTransform;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.SimCameraConfig;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.VisionSource;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.VisionSubsystemConfig;

public class VisionConstants {
  public static final Optional<VisionSystemSim> aprilTagSim =
      Constants.currentMode == Mode.SIM
          ? Optional.of(new VisionSystemSim("AprilTagSim"))
          : Optional.empty();

  public static final Optional<VisionSystemSim> objectDetectionSim =
      Constants.currentMode == Mode.SIM
          ? Optional.of(new VisionSystemSim("ObjectDetectionSim"))
          : Optional.empty();

  public static final ObjectDetectionCameraConfig intakeFuelDetection =
      new ObjectDetectionCameraConfig(
          new VisionSource(
              "DangerCam",
              RobotRelativeTransform.builder()
                  .forwardPositive(Inches.of(-24.747))
                  .leftPositive(Inches.of(0.696))
                  .upPositive(Inches.of(11.827))
                  .rollCCWPositive(Degrees.zero())
                  .pitchCCWPositive(Degrees.of(15.0))
                  .yawCCWPositive(Degrees.of(180))
                  .build()),
          SimCameraConfig.ARDUCAM_OV9782_100,
          List.of(ObjectDetectionStrategy.QUANTITY, ObjectDetectionStrategy.POSE_TRACKING));

  //   public static final ObjectDetectionCameraConfig hopperFuelDetection =
  //       new ObjectDetectionCameraConfig(
  //           new VisionSource(
  //               "ColemanCam",
  //               RobotRelativeTransform.builder()
  //                   .forwardPositive(Inches.of(4.388))
  //                   .leftPositive(Inches.of(9.936))
  //                   .upPositive(Inches.of(18.938))
  //                   .rollCCWPositive(Degrees.zero())
  //                   .pitchCCWPositive(Degrees.of(30.0))
  //                   .yawCCWPositive(Degrees.of(-135.0))
  //                   .build()),
  //           SimCameraConfig.ARDUCAM_OV9782_100,
  //           List.of(ObjectDetectionStrategy.QUANTITY));

  public static final List<AprilTagCameraConfig> aprilTagCamerasConfigs =
      List.of(
          new AprilTagCameraConfig(
              new VisionSource(
                  "LilCamThatCould",
                  RobotRelativeTransform.builder()
                      .forwardPositive(Inches.of(5.632))
                      .leftPositive(Inches.of(-12.190))
                      .upPositive(Inches.of(7.013))
                      .rollCCWPositive(Degrees.zero())
                      .pitchCCWPositive(Degrees.of(-20.0))
                      .yawCCWPositive(Degrees.of(-120))
                      .build()),
              SimCameraConfig.THRIFTY_CAM_95_720p,
              List.of(PoseEstimationMethod.MULTI_TAG)),
          new AprilTagCameraConfig(
              new VisionSource(
                  "ShooterCam",
                  RobotRelativeTransform.builder()
                      .forwardPositive(Inches.of(6.456))
                      .leftPositive(Inches.of(3.625))
                      .upPositive(Inches.of(19.455))
                      .rollCCWPositive(Degrees.zero())
                      .pitchCCWPositive(Degrees.of(-22.5))
                      .yawCCWPositive(Degrees.zero())
                      .build()),
              SimCameraConfig.THRIFTY_CAM_80,
              List.of(PoseEstimationMethod.MULTI_TAG)),
          new AprilTagCameraConfig(
              new VisionSource(
                  "BigCamThatWont",
                  RobotRelativeTransform.builder()
                      .forwardPositive(Inches.of(1.132))
                      .leftPositive(Inches.of(12.190))
                      .upPositive(Inches.of(7.013))
                      .rollCCWPositive(Degrees.zero())
                      .pitchCCWPositive(Degrees.of(-20.0))
                      .yawCCWPositive(Degrees.of(120))
                      .build()),
              SimCameraConfig.THRIFTY_CAM_95_720p,
              List.of(PoseEstimationMethod.MULTI_TAG))
          //   new AprilTagCameraConfig(
          //       new VisionSource(
          //           "EanCam",
          //           RobotRelativeTransform.builder()
          //               .forwardPositive(Inches.of(5.971))
          //               .leftPositive(Inches.of(8.750))
          //               .upPositive(Inches.of(19.750))
          //               .rollCCWPositive(Degrees.zero())
          //               .pitchCCWPositive(Degrees.zero())
          //               .yawCCWPositive(Degrees.of(180))
          //               .build()),
          //       SimCameraConfig.THRIFTY_CAM_95_720p,
          //       List.of(PoseEstimationMethod.MULTI_TAG))
          );

  public static final int noAmbiguity = -100;

  public static final VisionSubsystemConfig visionSubsystemConfig =
      VisionSubsystemConfig.builder()
          .aprilTagFieldLayout(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded))
          .aprilTagCameraConfigs(aprilTagCamerasConfigs)
          .robotHeadingSupplier(BobotState::getRawGyroHeading)
          .globalPoseSupplier(BobotState::getGlobalPose)
          .offerGlobalPoseObservation(BobotState::offerGlobalVisionObservation)
          .offerTrigPoseObservation(BobotState::offerTrigVisionObservation)
          .offerConstrainedPoseObservation(BobotState::offerConstrainedVisionObservation)
          .ambiguityCutoff(0.05)
          .singleTagPoseCutoff(Meters.of(4))
          .constrainedSolvePnpParams(new ConstrainedSolvepnpParams(true, 0))
          // The standard deviations of our vision estimated poses, which affect correction rate
          // (Fake values. Experiment and determine estimation noise on an actual robot.)
          .noStdDevs(VecBuilder.fill(0, 0, 0))
          .singleTagStdDevs(VecBuilder.fill(4, 4, Double.MAX_VALUE))
          .multiTagStdDevs(VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE))
          .tagStdDevs(VecBuilder.fill(0, 0, Double.MAX_VALUE))
          .build();

  public static final double fuelHeightMeters = 0.150;
  public static final double fuelRadiusMeters = fuelHeightMeters / 2.0;
  public static final double trackingMemorySeconds = 2.0;
}
