package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonomousHelpers {
  public static Command driveToInitialPose(String initialPathName) {

    // Command pathFindingCommand = Commands.none();
    Command pathFindingCommand;
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(initialPathName);
      Pose2d startingPose = path.getStartingHolonomicPose().get();
      // Create the constraints to use while pathfinding
      PathConstraints constraints =
          new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

      pathFindingCommand =
          AutoBuilder.pathfindToPose(
              startingPose, constraints, 0.0 // Goal end velocity in meters/sec
              );

    } catch (Exception e) {
      DriverStation.reportError("oopsie woopsie" + e.getMessage(), e.getStackTrace());
      pathFindingCommand = Commands.none();
    }
    return pathFindingCommand;
  }
}
