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

  public static final List<AprilTagCameraConfig> aprilTagCamerasConfigs =
      List.of(
          new AprilTagCameraConfig(
              new VisionSource(
                  "FrontLeftModule",
                  RobotRelativeTransform.builder()
                      .forwardPositive(Inches.of(11.198))
                      .leftPositive(Inches.of(10.802))
                      .upPositive(Inches.of(9.581))
                      .rollCCWPositive(Degrees.zero())
                      .pitchCCWPositive(Degrees.of(-30.0))
                      .yawCCWPositive(Degrees.of(115))
                      .build()),
              SimCameraConfig.THRIFTY_CAM_90,
              List.of(PoseEstimationMethod.MULTI_TAG)),
          new AprilTagCameraConfig(
              new VisionSource(
                  "FrontRightModule",
                  RobotRelativeTransform.builder()
                      .forwardPositive(Inches.of(10.974))
                      .leftPositive(Inches.of(-11.056))
                      .upPositive(Inches.of(9.536))
                      .rollCCWPositive(Degrees.zero())
                      .pitchCCWPositive(Degrees.of(-17.5))
                      .yawCCWPositive(Degrees.of(45))
                      .build()),
              SimCameraConfig.THRIFTY_CAM_90,
              List.of(PoseEstimationMethod.MULTI_TAG)),
          new AprilTagCameraConfig(
              new VisionSource(
                  "RightRearView",
                  RobotRelativeTransform.builder()
                      .forwardPositive(Inches.of(5.632))
                      .leftPositive(Inches.of(-12.190))
                      .upPositive(Inches.of(8.013))
                      .rollCCWPositive(Degrees.zero())
                      .pitchCCWPositive(Degrees.of(-20.0))
                      .yawCCWPositive(Degrees.of(-120))
                      .build()),
              SimCameraConfig.THRIFTY_CAM_90,
              List.of(PoseEstimationMethod.MULTI_TAG)),
          new AprilTagCameraConfig(
              new VisionSource(
                  "ShooterForward",
                  RobotRelativeTransform.builder()
                      .forwardPositive(Inches.of(6.691))
                      .leftPositive(Inches.of(3.250))
                      .upPositive(Inches.of(19.256))
                      .rollCCWPositive(Degrees.zero())
                      .pitchCCWPositive(Degrees.of(-22.5))
                      .yawCCWPositive(Degrees.zero())
                      .build()),
              SimCameraConfig.THRIFTY_CAM_90,
              List.of(
                  PoseEstimationMethod.MULTI_TAG,
                  PoseEstimationMethod.TRIG,
                  PoseEstimationMethod.CONSTRAINED)));

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
}
