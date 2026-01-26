package frc.robot.bobot_state;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.PoseObservation;
import org.robotzgarage.frc2026.util.VirtualSubsystem;

public class BobotState extends VirtualSubsystem {
  private static final String logRoot = "BobotState/";

  private static final Queue<PoseObservation> globalPoseObservations =
      new LinkedBlockingQueue<>(20);
  private static final Queue<PoseObservation> constrainedPoseObservations =
      new LinkedBlockingQueue<>(20);

  private static Pose2d globalPose = new Pose2d();
  private static Pose2d constrainedPose = new Pose2d();

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

  public static void updateGlobalPose(Pose2d pose) {
    BobotState.globalPose = pose;
  }

  public static void updateConstrainedPose(Pose2d pose) {
    BobotState.constrainedPose = pose;
  }

  public static Pose2d getGlobalPose() {
    return BobotState.globalPose;
  }

  public static Pose2d getConstrainedPose() {
    return BobotState.constrainedPose;
  }

  public void periodic() {}

  public void simulationPeriodic() {}
}
