package org.robotzgarage.frc2026.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.function.Supplier;
import lombok.Builder;
import lombok.NonNull;
import org.photonvision.simulation.SimCameraProperties;

public final class VisionTypes {
  @Builder
  public record RobotRelativeTransform(
      @NonNull Distance forwardPositive,
      @NonNull Distance leftPositive,
      @NonNull Distance upPositive,
      @NonNull Angle rollPositive,
      @NonNull Angle pitchPositive,
      @NonNull Angle yawPositive) {
    public Transform3d getTransform() {
      return new Transform3d(
          new Translation3d(
              forwardPositive.in(Meters), leftPositive.in(Meters), upPositive.in(Meters)),
          new Rotation3d(
              -rollPositive.in(Radians), -pitchPositive.in(Radians), -yawPositive.in(Radians)));
    }
  }

  public record PrototypingVisionSource(
      String name, Supplier<RobotRelativeTransform> robotToCamera) {}

  public record VisionSource(String name, RobotRelativeTransform robotToCamera) {}

  public record AprilTagStruct(int fiducialId, Pose3d pose) {}

  public static final record AprilTagCameraConfig(VisionSource source, SimCameraConfig simConfig) {}

  public static enum PoseEstimationMethod {
    MULTI_TAG,
    SINGLE_TAG,
    TRIG,
    CONSTRAINED
  }

  public record PoseObservation(
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

    public static SimCameraConfig THRIFTY_CAM_90 =
        new SimCameraConfig(
            Calibration.builder()
                .widthPx(1600)
                .heightPx(1304)
                .fov(Rotation2d.fromDegrees(90))
                .fps(60)
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
  }
}
