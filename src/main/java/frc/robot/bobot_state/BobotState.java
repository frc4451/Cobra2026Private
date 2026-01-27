package frc.robot.bobot_state;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import lombok.Getter;
import lombok.Setter;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.PoseObservation;
import org.robotzgarage.frc2026.util.VirtualSubsystem;

public class BobotState extends VirtualSubsystem {
  private static final String logRoot = "BobotState/";

  private static final Queue<PoseObservation> globalPoseObservations =
      new LinkedBlockingQueue<>(20);
  private static final Queue<PoseObservation> constrainedPoseObservations =
      new LinkedBlockingQueue<>(20);

  @Getter @Setter private static Pose2d globalPose = new Pose2d();
  @Getter @Setter private static Pose2d constrainedPose = new Pose2d();
  @Getter @Setter private static Rotation2d rawGyroHeading = new Rotation2d();

  public static void offerGlobalVisionObservation(PoseObservation observation) {
    BobotState.globalPoseObservations.offer(observation);
  }

  public static Queue<PoseObservation> getGlobalVisionObservations() {
    return BobotState.globalPoseObservations;
  }

  public static void offerConstrainedVisionObservation(PoseObservation observation) {
    BobotState.constrainedPoseObservations.offer(observation);
  }

  public static Queue<PoseObservation> getConstrainedVisionObservations() {
    return BobotState.constrainedPoseObservations;
  }

  public void periodic() {}

  public void simulationPeriodic() {}
}
