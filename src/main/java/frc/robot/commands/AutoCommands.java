package frc.robot.commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.robotzgarage.frc2026.math.distance_calc.PoseUtils;

public class AutoCommands {
  public static Command interpolationDistanceHelper(
      Drive drive,
      Pose2d pose,
      DoubleSupplier perpendicularOffsetFeet,
      DoubleSupplier parallelOffsetFeet) {
    if (Constants.currentMode != Mode.SIM) {
      throw new UnsupportedOperationException("You can only run this command in Sim");
    }
    PathConstraints constraints =
        new PathConstraints(
            TunerConstants.kSpeedAt12Volts,
            MetersPerSecondPerSecond.of(12),
            RotationsPerSecond.of(1),
            RotationsPerSecondPerSecond.of(1));

    return new DeferredCommand(
        () -> {
          Pose2d offsetPose =
              PoseUtils.getOffsetPose(
                  pose,
                  Feet.of(parallelOffsetFeet.getAsDouble()),
                  Feet.of(perpendicularOffsetFeet.getAsDouble()));
          return AutoBuilder.pathfindToPose(offsetPose, constraints, 0.0);
        },
        Set.of(drive));
  }
}
