package org.robotzgarage.frc2026.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import lombok.Builder;
import lombok.NonNull;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.simulation.SimCameraProperties;

public final class VisionTypes {
  @Builder
  public record RobotRelativeTransform(
      /** From center of robot towards "front" */
      @NonNull Distance forwardPositive,
      /** From center of robot towards "left" */
      @NonNull Distance leftPositive,
      /** From center of robot (on the floor) towards "up". */
      @NonNull Distance upPositive,
      /** Rotation over "X", like rolling a plane while turning. CCW is Positive. */
      @NonNull Angle rollCCWPositive,
      /** Rotation over "Y", like angling a plane's nose up. CCW is Positive */
      @NonNull Angle pitchCCWPositive,
      /** Rotation over "Z", like rotating a robot on the floor. CCW is Positive. */
      @NonNull Angle yawCCWPositive) {
    public Transform3d getTransform() {
      return new Transform3d(
          new Translation3d(
              forwardPositive.in(Meters), leftPositive.in(Meters), upPositive.in(Meters)),
          new Rotation3d(
              rollCCWPositive.in(Radians),
              pitchCCWPositive.in(Radians),
              yawCCWPositive.in(Radians)));
    }
  }

  public record VisionSource(String name, RobotRelativeTransform robotToCamera) {}

  public record AprilTagStruct(int fiducialId, Pose3d pose) {
    public static AprilTagStruct fromFiducialId(int fiducialId, AprilTagFieldLayout fieldLayout) {
      return new AprilTagStruct(fiducialId, fieldLayout.getTagPose(fiducialId).get());
    }
  }

  public static final record ObjectDetectionCameraConfig(
      VisionSource source, SimCameraConfig simCameraConfig) {}

  @Builder
  public static final record ObjectDetectionTrackedResult(
      @NonNull Rotation2d pitch, @NonNull Rotation2d yaw, double area) {}

  public static final record AprilTagCameraConfig(
      VisionSource source,
      SimCameraConfig simConfig,
      List<PoseEstimationMethod> estimationMethods) {}

  public static enum PoseEstimationMethod {
    MULTI_TAG,
    SINGLE_TAG,
    TRIG,
    CONSTRAINED
  }

  /**
   * Helper data struct to provide all variables known to be used for vision pipelines by the
   * PhotonVision process.
   */
  @Builder
  public record VisionSubsystemConfig(
      @NonNull AprilTagFieldLayout aprilTagFieldLayout,
      @NonNull List<AprilTagCameraConfig> aprilTagCameraConfigs,
      @NonNull Supplier<Rotation2d> robotHeadingSupplier,
      @NonNull Supplier<Pose2d> globalPoseSupplier,
      @NonNull Consumer<PoseObservation> offerGlobalPoseObservation,
      @NonNull Consumer<PoseObservation> offerConstrainedPoseObservation,
      @NonNull Consumer<PoseObservation> offerTrigPoseObservation,
      Double ambiguityCutoff,
      Distance singleTagPoseCutoff,
      ConstrainedSolvepnpParams constrainedSolvePnpParams,
      Matrix<N3, N1> noStdDevs,
      Matrix<N3, N1> singleTagStdDevs,
      Matrix<N3, N1> multiTagStdDevs,
      Matrix<N3, N1> tagStdDevs) {
    public VisionSubsystemConfig {
      if (ambiguityCutoff == null) {
        ambiguityCutoff = 0.05;
      }
      if (singleTagPoseCutoff == null) {
        singleTagPoseCutoff = Meters.of(4);
      }
      if (constrainedSolvePnpParams == null) {
        constrainedSolvePnpParams = new ConstrainedSolvepnpParams(true, 0);
      }
      if (noStdDevs == null) {
        noStdDevs = VecBuilder.fill(0, 0, 0);
      }
      if (singleTagStdDevs == null) {
        singleTagStdDevs = VecBuilder.fill(4, 4, Double.MAX_VALUE);
      }
      if (multiTagStdDevs == null) {
        multiTagStdDevs = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
      }
      if (tagStdDevs == null) {
        tagStdDevs = VecBuilder.fill(0, 0, Double.MAX_VALUE);
      }
    }
  }

  public record PoseObservation(
      String source,
      Pose3d robotPose,
      double timestampSeconds,
      double ambiguity,
      int id,
      Matrix<N3, N1> stdDevs,
      PoseEstimationMethod method) {}

  @Builder
  public record SimCameraConfig(Calibration calib, CalibrationError calibError, Latency latency) {
    @Builder
    public static record Calibration(int widthPx, int heightPx, Rotation2d fov, double fps) {}

    @Builder
    public static record CalibrationError(double avgErrorPx, double errorStdDevPx) {}

    @Builder
    public static record Latency(double avgLatencyMs, double latencyStdDevMs) {}

    public SimCameraProperties apply(SimCameraProperties props) {
      props.setCalibration(calib.widthPx, calib.heightPx, calib.fov);
      props.setFPS(calib.fps);
      props.setCalibError(calibError.avgErrorPx, calibError.errorStdDevPx);
      props.setAvgLatencyMs(latency.avgLatencyMs);
      props.setLatencyStdDevMs(latency.latencyStdDevMs);

      return props;
    }

    private static final CalibrationError defaultCalibrationError =
        CalibrationError.builder().avgErrorPx(0.25).errorStdDevPx(0.08).build();
    private static final Latency stockLatency =
        Latency.builder().avgLatencyMs(20).latencyStdDevMs(5).build();

    /** Default simulated camera configuration for a Thrify Cam * */
    public static SimCameraConfig THRIFTY_CAM_STOCK =
        new SimCameraConfig(
            Calibration.builder()
                .widthPx(1600)
                .heightPx(1304)
                .fov(Rotation2d.fromDegrees(55))
                .fps(60)
                .build(),
            defaultCalibrationError,
            stockLatency);

    public static SimCameraConfig THRIFTY_CAM_65 =
        new SimCameraConfig(
            Calibration.builder()
                .widthPx(1600)
                .heightPx(1304)
                .fov(Rotation2d.fromDegrees(65))
                .fps(60)
                .build(),
            defaultCalibrationError,
            stockLatency);
    public static SimCameraConfig THRIFTY_CAM_70 =
        new SimCameraConfig(
            Calibration.builder()
                .widthPx(1600)
                .heightPx(1304)
                .fov(Rotation2d.fromDegrees(70))
                .fps(60)
                .build(),
            defaultCalibrationError,
            stockLatency);

    public static SimCameraConfig THRIFTY_CAM_80 =
        new SimCameraConfig(
            Calibration.builder()
                .widthPx(1600)
                .heightPx(1304)
                .fov(Rotation2d.fromDegrees(80))
                .fps(60)
                .build(),
            defaultCalibrationError,
            stockLatency);

    public static SimCameraConfig THRIFTY_CAM_95_MAX =
        new SimCameraConfig(
            Calibration.builder()
                .widthPx(1600)
                .heightPx(1304)
                .fov(Rotation2d.fromDegrees(95))
                .fps(60)
                .build(),
            defaultCalibrationError,
            stockLatency);

    public static SimCameraConfig THRIFTY_CAM_95_720p =
        new SimCameraConfig(
            Calibration.builder()
                .widthPx(1280)
                .heightPx(720)
                .fov(Rotation2d.fromDegrees(95))
                .fps(90)
                .build(),
            defaultCalibrationError,
            stockLatency);

    public static SimCameraConfig ARDUCAM_OV9281_45 =
        new SimCameraConfig(
            Calibration.builder()
                .widthPx(1280)
                .heightPx(720)
                .fov(Rotation2d.fromDegrees(45))
                .fps(60)
                .build(),
            defaultCalibrationError,
            stockLatency);

    public static SimCameraConfig ARDUCAM_OV9281_55 =
        new SimCameraConfig(
            Calibration.builder()
                .widthPx(1280)
                .heightPx(720)
                .fov(Rotation2d.fromDegrees(55))
                .fps(60)
                .build(),
            defaultCalibrationError,
            stockLatency);

    public static SimCameraConfig ARDUCAM_OV9281_65 =
        new SimCameraConfig(
            Calibration.builder()
                .widthPx(1280)
                .heightPx(720)
                .fov(Rotation2d.fromDegrees(65))
                .fps(60)
                .build(),
            defaultCalibrationError,
            stockLatency);

    public static SimCameraConfig ARDUCAM_OV9281_95 =
        new SimCameraConfig(
            Calibration.builder()
                .widthPx(1280)
                .heightPx(720)
                .fov(Rotation2d.fromDegrees(95))
                .fps(60)
                .build(),
            defaultCalibrationError,
            stockLatency);

    public static SimCameraConfig ARDUCAM_OV9782_100 =
        new SimCameraConfig(
            Calibration.builder()
                .widthPx(640)
                .heightPx(480)
                .fov(Rotation2d.fromDegrees(100))
                .fps(60)
                .build(),
            defaultCalibrationError,
            stockLatency);

    public static SimCameraConfig ARDUCAM_OV9281_95_ROTATED =
        new SimCameraConfig(
            Calibration.builder()
                .widthPx(720)
                .heightPx(1280)
                .fov(Rotation2d.fromDegrees(95))
                .fps(60)
                .build(),
            defaultCalibrationError,
            stockLatency);
  }
}
