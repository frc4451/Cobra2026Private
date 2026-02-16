package frc.robot.autonomous.bline;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.bobot_state.BobotState;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.Drive;

public class BLineManager {
  // Create a reusable builder with your robot's configuration
  private final FollowPath.Builder pathBuilder;

  public BLineManager(Drive drive) {
    pathBuilder =
        new FollowPath.Builder(
                drive, // The drive subsystem to require
                BobotState::getGlobalPose, // Supplier for current robot pose
                drive::getChassisSpeeds, // Supplier for current speeds
                drive::runVelocity, // Consumer to drive the robot
                new PIDController(5.0, 0.0, 0.0), // Translation PID
                new PIDController(3.0, 0.0, 0.0), // Rotation PID
                new PIDController(2.0, 0.0, 0.0) // Cross-track PID
                )
            .withDefaultShouldFlip(); // Auto-flip for red alliance
    // .withPoseReset(drive::resetPose);  // Reset odometry at path start
  }

  public Command getSampleBLineAutoRoutine() {
    Path examplePath = new Path("example_a");

    return Commands.sequence(pathBuilder.build(examplePath));
  }
}
