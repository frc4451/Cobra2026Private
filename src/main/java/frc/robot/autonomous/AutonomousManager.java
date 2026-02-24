package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.bobot_state.BobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutonomousManager {
  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;

  private static final double rotationToleranceDegrees = 1.0;
  private static final Time MAX_TIMEOUT = Seconds.of(20);

  public AutonomousManager(Drive drive, Intake intakeV1, Shooter shooterV1) {
    this.drive = drive;
    this.intake = intakeV1;
    this.shooter = shooterV1;
  }

  /**
   * "On-the-fly" "Rotate-in-Place"
   *
   * <p>This uses "snapToAngle" to lock the rotation of the robot to the hub
   */
  private Command rotateToHub() {
    return DriveCommands.snapToAngle(
        drive, () -> BobotState.getHubVARC().getRotationTarget(), rotationToleranceDegrees);
  }

  private Command handleShot(double velocity, double tolerance, double emptyTime) {
    AngularVelocity shooterVelocity = RotationsPerSecond.of(velocity);
    AngularVelocity shooterTolerance = RotationsPerSecond.of(tolerance);
    Time estimatedShooterEmptyTime = Seconds.of(emptyTime);

    return Commands.sequence(
        shooter
            // set velocity to shooter at with torque current
            .runShooterVelocityTorqueCurrent(shooterVelocity)
            .until(
                // wait to start tower, until shooter is at velocity
                shooter.isShooterFlywheelNearRPS(shooterVelocity, shooterTolerance)),
        Commands.sequence(
            // shoots shooter for the estimated empty time
            shooter
                .runShooterVelocityTorqueCurrent(shooterVelocity)
                .withTimeout(estimatedShooterEmptyTime),
            shooter.stop()));
  }

  /** deploys intake to set position to intake at */
  private Command deployIntake() {
    return Commands.sequence(intake.endeffectorGoal().until(intake.isNearGoal(Degrees.of(5))));
  }

  /** retracks intake to starting position */
  private Command retrackeIntake() {
    return Commands.sequence(intake.endeffectorHome().until(intake.isNearGoal(Degrees.of(5))));
  }

  private Command BLineTrajectory(String name) {
    return Commands.sequence(drive.getBLineBuilder().build(new Path(name)));
  }

  public Command himithy() {
    return Commands.sequence(
        BLineTrajectory("SnG-LT-1"),
        this.deployIntake(),
        BLineTrajectory("LT-SimpleIntake-1"),
        BLineTrajectory("SimpleIntake-CrossField"));
  }
}
