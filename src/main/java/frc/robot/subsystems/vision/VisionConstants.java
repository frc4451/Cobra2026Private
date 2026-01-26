package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.simulation.VisionSystemSim;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.AprilTagCameraConfig;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.RobotRelativeTransform;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.SimCameraConfig;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.VisionSource;

public class VisionConstants {
  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

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
                      .forwardPositive(Inches.of(-5.671))
                      .leftPositive(Inches.of(11.237))
                      .upPositive(Inches.of(7.991))
                      .rollPositive(Degrees.zero())
                      .pitchPositive(Degrees.of(20.0))
                      .yawPositive(Degrees.of(30))
                      .build()),
              SimCameraConfig.THRIFTY_CAM_80),
          // FLI
          new AprilTagCameraConfig(
              new VisionSource(
                  "FrontLeft",
                  RobotRelativeTransform.builder()
                      .forwardPositive(Inches.of(6.354))
                      .leftPositive(Inches.of(11.143))
                      .upPositive(Inches.of(8.058))
                      .rollPositive(Degrees.zero())
                      .pitchPositive(Degrees.of(17.5))
                      .yawPositive(Degrees.of(10))
                      .build()),
              SimCameraConfig.THRIFTY_CAM_90),
          // FRI
          new AprilTagCameraConfig(
              new VisionSource(
                  "FrontRight",
                  RobotRelativeTransform.builder()
                      .forwardPositive(Inches.of(6.354))
                      .leftPositive(Inches.of(-11.143))
                      .upPositive(Inches.of(8.058))
                      .rollPositive(Degrees.zero())
                      .pitchPositive(Degrees.of(17.5))
                      .yawPositive(Degrees.of(10))
                      .build()),
              SimCameraConfig.THRIFTY_CAM_90),
          // FRO
          new AprilTagCameraConfig(
              new VisionSource(
                  "RearRight",
                  RobotRelativeTransform.builder()
                      .forwardPositive(Inches.of(-5.671))
                      .leftPositive(Inches.of(-11.237))
                      .upPositive(Inches.of(7.991))
                      .rollPositive(Degrees.zero())
                      .pitchPositive(Degrees.of(20.0))
                      .yawPositive(Degrees.of(30))
                      .build()),
              SimCameraConfig.THRIFTY_CAM_80));

  //   public static final List<AprilTagCameraConfig> aprilTagCamerasConfigs = riptideConfigs;

  public static final double ambiguityCutoff = 0.05;
  public static final double singleTagPoseCutoffMeters = 4.0;

  public static final int noAmbiguity = -100;

  public static final Optional<ConstrainedSolvepnpParams> constrainedSolvePnpParams =
      Optional.of(new ConstrainedSolvepnpParams(true, 0));

  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> noStdDevs = VecBuilder.fill(0, 0, 0);
  public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, Double.MAX_VALUE);
  public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);

  public static final Matrix<N3, N1> tagStdDevs = VecBuilder.fill(0, 0, Double.MAX_VALUE);
}
