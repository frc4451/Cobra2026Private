package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.bobot_state.BobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutonomousManager {
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Feeder feeder;

  private static final double rotationToleranceDegrees = 3.0;
  // private static final Time MAX_TIMEOUT = Seconds.of(20);

  public AutonomousManager(Drive drive, Shooter shooter, Intake intake, Feeder feeder) {
    this.drive = drive;
    this.shooter = shooter;
    this.intake = intake;
    this.feeder = feeder;
  }

  /**
   * "On-the-fly" "Rotate-in-Place"
   *
   * <p>This uses "snapToAngle" to lock the rotation of the robot to the hub
   */
  private Command rotateToHub() {
    // return drive.getBLineBuilder().build(
    //   new Path(
    //     new Path.Waypoint(null)
    //   )
    // )

    return DriveCommands.snapToAngle(
        drive, () -> BobotState.getHubVARC().getRotationTarget(), rotationToleranceDegrees);
  }

  private Command handleShotInterpolation(String prefix, double emptyTime, double squeezeTime) {
    String formattedPrefix = prefix + ": ";

    return Commands.sequence(
            Commands.print(formattedPrefix + "start shooter"),
            Commands.deadline(
                    shooter.runShooterToHub().withTimeout(Seconds.of(emptyTime)),
                    feeder.runFeederVoltage())
                .alongWith(Commands.waitSeconds(squeezeTime).andThen(intake.wiggelsqueeze())),
            feeder.stop(),
            shooter.stop())
        .beforeStarting(Commands.print(formattedPrefix + "start"))
        .andThen(Commands.print(formattedPrefix + "end"));
  }

  private Command handleInterpolateShooter(String prefix, double emptyTime) {
    String formattedPrefix = prefix + ": ";

    return Commands.sequence(
            Commands.print(formattedPrefix + "start shooter"),
            Commands.deadline(
                shooter.runShooterToHub().withTimeout(Seconds.of(emptyTime)),
                feeder.runFeederVoltage()),
            shooter.stop(),
            feeder.stop())
        .beforeStarting(Commands.print(formattedPrefix + "start"))
        .andThen(Commands.print(formattedPrefix + "end"));
  }

  /** deploys intake to set position to intake at */
  private Command deployIntake() {
    return Commands.sequence(intake.endeffectorDeploy().until(intake.isNearGoal(Degrees.of(3))))
        .beforeStarting(Commands.print("deploy: start"))
        .andThen(Commands.print("deploy: end"));
  }

  /** deploys intake to set position to intake with time */
  private Command timedDeployIntake(double time) {
    return Commands.deadline(Commands.waitSeconds(time), intake.endeffectorDeploy())
        .beforeStarting(Commands.print("deploy: start"))
        .andThen(Commands.print("deploy: end"));
  }

  // /** retracks intake to starting position */
  // private Command retrackIntake() {
  //   return Commands.sequence(intake.endeffectorStart().until(intake.isNearGoal(Degrees.of(5))))
  //       .beforeStarting(Commands.print("retrack: start"))
  //       .andThen(Commands.print("retrack: end"));
  // }

  // /** lifts intake to an offset position for bump transversal */
  // private Command liftIntake() {
  //   return Commands.sequence(intake.endeffectorLift())
  //       .until(intake.isNearGoal(Degrees.of(5)))
  //       .beforeStarting(Commands.print("lift: start"))
  //       .andThen(Commands.print("lift: end"));
  // }

  // /** Sqeeze intake for shooting */
  // private Command squeezeIntake() {
  //   return Commands.sequence(intake.endeffectorSqueeze())
  //       .until(intake.isNearGoal(Degrees.of(5)))
  //       .beforeStarting(Commands.print("Squeeze: start"))
  //       .andThen(Commands.print("Squeeze: end"));
  // }

  private Command runIntake() {
    return intake.runRollersTorqueCurrent().beforeStarting(Commands.print("Intake rollers: start"));
  }

  private Command stopIntake() {
    return Commands.runOnce(() -> intake.stop())
        .beforeStarting(Commands.print("Intake rollers: stop"));
  }

  private Command BLineTrajectory(String name) {
    return Commands.sequence(drive.getBLineBuilder().build(new Path(name)))
        .beforeStarting(Commands.print(name + ": start"))
        .andThen(Commands.print(name + ": end"));
  }

  private Command BLineTrajectorywithIntake(String name) {
    return Commands.deadline(
            drive.getBLineBuilder().build(new Path(name)), intake.runRollersTorqueCurrent())
        .beforeStarting(Commands.print(name + ": start"))
        .andThen(Commands.sequence(Commands.print(name + ": end")), intake.stop());
  }

  private Command BLineTrajectorywithIntakeLift(String name, double intaketime) {
    return Commands.deadline(
            drive.getBLineBuilder().build(new Path(name)),
            Commands.deadline(Commands.waitSeconds(intaketime), intake.runRollersTorqueCurrent())
                .andThen(intake.endeffectorLift()))
        .beforeStarting(Commands.print(name + ": start"))
        .andThen(Commands.sequence(Commands.print(name + ": end")), intake.stop());
  }

  /** Follows {@code name} with hub rotation override */
  private Command BLineTrajectoryWithHubRotation(String name) {
    return drive
        .followPathWithRotationTarget(
            new Path(name), () -> BobotState.getHubVARC().getRotationTarget())
        .beforeStarting(Commands.print(name + ": start"))
        .andThen(Commands.print(name + ": end"));
  }

  /** Follows {@code name} with hub rotation override */
  private Command BLineTrajectorySOTM(String name, double time, Command shootCommand) {
    return Commands.deadline(
        Commands.waitSeconds(time),
        drive
            .followPathWithRotationTarget(
                new Path(name), () -> BobotState.getHubVARC().getRotationTarget())
            .beforeStarting(Commands.print(name + ": start"))
            .andThen(Commands.print(name + ": end")),
        shootCommand);
  }

  public Command himithyTake2() {
    return Commands.sequence(
            this.timedDeployIntake(0.8),
            BLineTrajectorywithIntakeLift("Himithy-1", 3.0),
            BLineTrajectorySOTM("Himithy-2", 5.0, handleShotInterpolation("shot 1", 5.0, 3.0)),
            this.deployIntake(),
            BLineTrajectorywithIntakeLift("Himithy-3", 3.5),
            BLineTrajectorySOTM("Himithy-2", 5.0, handleShotInterpolation("shot 2", 5.0, 3.0)))
        .beforeStarting(Commands.print("Path start"))
        .andThen(Commands.print("Path end"));
  }

  public Command simithySOTMDemo() {
    return Commands.sequence(
        Commands.print("Starting  Simithy"),
        BLineTrajectoryWithHubRotation("Simithy-1"),
        Commands.print("Finishing Simithy"));
  }

  /** Fork of {@link #bimithySmiley} that overrides rotation to face the hub (or attempts to) */
  public Command bimithySmileyHubRotation() {
    return Commands.sequence(
            this.deployIntake(),
            Commands.waitSeconds(1.0).until(intake.isNearGoal(Degrees.of(8.0))),
            BLineTrajectorywithIntakeLift("Bimithy-1-SOTM", 3.0),
            this.deployIntake(),
            Commands.deadline(
                Commands.waitTime(Seconds.of(4.0)),
                BLineTrajectoryWithHubRotation("Bimithy-2-SOTM"),
                this.handleInterpolateShooter("shot 1", 4.0),
                intake.flickIntakeRepeatedly(
                    Degrees.of(0.0),
                    Degrees.of(45.0),
                    Degrees.of(0.0),
                    Degrees.of(5.0),
                    Seconds.of(4.0))),
            this.deployIntake(),
            BLineTrajectorywithIntakeLift("Bimithy-3-SOTM", 4.25),
            this.rotateToHub(),
            Commands.deadline(
                Commands.waitTime(Seconds.of(20.0)),
                this.handleInterpolateShooter("shot 2", 20.0),
                intake.flickIntakeNumberOfTimes(
                    Degrees.of(20.0), Degrees.of(80.0), Degrees.of(0.0), Degrees.of(5.0), 3)))
        .beforeStarting(Commands.print("Path start"))
        .andThen(Commands.print("Path end"));
  }

  public Command bimithySmiley() {
    return Commands.sequence(
            Commands.sequence(
                this.deployIntake(),
                BLineTrajectorywithIntakeLift("Bimithy-1", 5.0),
                this.rotateToHub(),
                this.handleShotInterpolation("shot 1", 4.0, 2.0)),
            this.deployIntake(),
            BLineTrajectorywithIntake("Bimithy-2"),
            BLineTrajectorywithIntake("Bimithy-3"),
            this.rotateToHub(),
            this.handleShotInterpolation("shot 2", 4.0, 2.0))
        .beforeStarting(Commands.print("Path start"))
        .andThen(Commands.print("Path end"));
  }

  public Command bline1ft() {
    return Commands.sequence(BLineTrajectory("1ft-bline"));
  }

  public Command bline2ft() {
    return Commands.sequence(BLineTrajectory("2ft-bline"));
  }

  public Command test() {
    return Commands.sequence(BLineTrajectorywithIntake("Himithy-2"));
  }
}
