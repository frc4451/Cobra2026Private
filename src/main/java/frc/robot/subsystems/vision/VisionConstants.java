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
          // FLO
          new AprilTagCameraConfig(
              new VisionSource(
                  "RearLeft",
                  RobotRelativeTransform.builder()
                      .forwardPositive(Inches.of(11.0))
                      .leftPositive(Inches.of(11.0))
                      .upPositive(Inches.of(6.813))
                      .rollPositive(Degrees.zero())
                      .pitchPositive(Degrees.of(20.0))
                      .yawPositive(Degrees.of(-110))
                      .build()),
              SimCameraConfig.ARDUCAM_OV9281_95),
          // FLI
          new AprilTagCameraConfig(
              new VisionSource(
                  "FrontLeft",
                  RobotRelativeTransform.builder()
                      .forwardPositive(Inches.of(11.0))
                      .leftPositive(Inches.of(11.0))
                      .upPositive(Inches.of(6.813))
                      .rollPositive(Degrees.zero())
                      .pitchPositive(Degrees.of(27.5))
                      .yawPositive(Degrees.of(20))
                      .build()),
              SimCameraConfig.THRIFTY_CAM_90),
          // FRI
          new AprilTagCameraConfig(
              new VisionSource(
                  "FrontRight",
                  RobotRelativeTransform.builder()
                      .forwardPositive(Inches.of(11.0))
                      .leftPositive(Inches.of(-11.0))
                      .upPositive(Inches.of(6.813))
                      .rollPositive(Degrees.zero())
                      .pitchPositive(Degrees.of(17.5))
                      .yawPositive(Degrees.of(-20))
                      .build()),
              SimCameraConfig.THRIFTY_CAM_90),
          // FRO
          new AprilTagCameraConfig(
              new VisionSource(
                  "RearRight",
                  RobotRelativeTransform.builder()
                      .forwardPositive(Inches.of(11.0))
                      .leftPositive(Inches.of(-11.0))
                      .upPositive(Inches.of(6.813 + 3.0))
                      .rollPositive(Degrees.zero())
                      .pitchPositive(Degrees.of(40.0))
                      .yawPositive(Degrees.of(130))
                      .build()),
              SimCameraConfig.ARDUCAM_OV9281_95));

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
