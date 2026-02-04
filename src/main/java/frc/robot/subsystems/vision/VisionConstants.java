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
                  "FrontRightModule",
                  RobotRelativeTransform.builder()
                      .forwardPositive(Inches.of(11.0))
                      .leftPositive(Inches.of(-11.0))
                      .upPositive(Inches.of(6.813))
                      .rollCCWPositive(Degrees.zero())
                      .pitchCCWPositive(Degrees.of(-17.5))
                      .yawCCWPositive(Degrees.of(20))
                      .build()),
              SimCameraConfig.THRIFTY_CAM_90),
          new AprilTagCameraConfig(
              new VisionSource(
                  "FrontLeftModule",
                  RobotRelativeTransform.builder()
                      .forwardPositive(Inches.of(11.0))
                      .leftPositive(Inches.of(11.0))
                      .upPositive(Inches.of(6.813))
                      .rollCCWPositive(Degrees.zero())
                      .pitchCCWPositive(Degrees.of(-17.5))
                      .yawCCWPositive(Degrees.of(120))
                      .build()),
              SimCameraConfig.THRIFTY_CAM_90),
          // FRO
          new AprilTagCameraConfig(
              new VisionSource(
                  "DriveRailBackwards",
                  RobotRelativeTransform.builder()
                      .forwardPositive(Inches.of(5.521))
                      .leftPositive(Inches.of(-12.215))
                      .upPositive(Inches.of(8.022))
                      .rollCCWPositive(Degrees.zero())
                      .pitchCCWPositive(Degrees.of(-15.0))
                      .yawCCWPositive(Degrees.of(135))
                      .build()),
              SimCameraConfig.THRIFTY_CAM_90),
          // Cobra Testing
          new AprilTagCameraConfig(
              new VisionSource(
                  "PowerPanelForward",
                  RobotRelativeTransform.builder()
                      .forwardPositive(Inches.of(6.824))
                      .leftPositive(Inches.of(8.75))
                      .upPositive(Inches.of(20.272))
                      .rollCCWPositive(Degrees.zero())
                      .pitchCCWPositive(Degrees.of(-15.0))
                      .yawCCWPositive(Degrees.of(-30.0))
                      .build()),
              SimCameraConfig.THRIFTY_CAM_90)
          //   new AprilTagCameraConfig(
          //       new VisionSource(
          //           "PowerPanelBackwards",
          //           RobotRelativeTransform.builder()
          //               .forwardPositive(Inches.of(6.824))
          //               .leftPositive(Inches.of(8.75))
          //               .upPositive(Inches.of(20.272))
          //               .rollCCWPositive(Degrees.zero())
          //               .pitchCCWPositive(Degrees.of(-15.0))
          //               .yawCCWPositive(Degrees.of(120.0))
          //               .build()),
          //       SimCameraConfig.THRIFTY_CAM_90)
          //   new AprilTagCameraConfig(
          //       new VisionSource(
          //           "PowerPanelBack",
          //           RobotRelativeTransform.builder()
          //               .forwardPositive(Inches.of(6.0))
          //               .leftPositive(Inches.of(11.5))
          //               .upPositive(Inches.of(20.250))
          //               .rollCCWPositive(Degrees.zero())
          //               .pitchPositive(Degrees.of(10.0))
          //               .yawCCWPositive(Degrees.of(135.0))
          //               .build()),
          //       SimCameraConfig.THRIFTY_CAM_90)
          //
          );

  public static final int noAmbiguity = -100;

  public static final VisionSubsystemConfig visionSubsystemConfig =
      VisionSubsystemConfig.builder()
          .aprilTagFieldLayout(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded))
          .aprilTagCameraConfigs(aprilTagCamerasConfigs)
          .robotHeadingSupplier(BobotState::getRawGyroHeading)
          .globalPoseSupplier(BobotState::getGlobalPose)
          .offerGlobalPoseObservation(BobotState::offerGlobalVisionObservation)
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
