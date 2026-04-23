package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.bobot_state.BobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousManager {
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Feeder feeder;

  private final String shouldStartOnBumpKey = "Autonomous/ShouldStartOnBump";
  private final LoggedDashboardChooser<Boolean> shouldStartOnBumpChooser;

  private static final double rotationToleranceDegrees = 3.0;
  // private static final Time MAX_TIMEOUT = Seconds.of(20);

  public AutonomousManager(Drive drive, Shooter shooter, Intake intake, Feeder feeder) {
    this.drive = drive;
    this.shooter = shooter;
    this.intake = intake;
    this.feeder = feeder;

    // Default that we start in the trench, but optionally shift the starting point to the
    // bump. Then follow the autonomous routine as we expect from BLine.
    SendableChooser<Boolean> bumpChooser = new SendableChooser<>();
    bumpChooser.setDefaultOption("NO", Boolean.FALSE);
    bumpChooser.addOption("YES", Boolean.TRUE);

    shouldStartOnBumpChooser = new LoggedDashboardChooser<>(shouldStartOnBumpKey, bumpChooser);
  }

  public boolean shouldStartOnBump() {
    boolean shouldStartOnBump = Boolean.TRUE.equals(shouldStartOnBumpChooser.get());
    return shouldStartOnBump;
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

  /** This uses "snapToAngle" to lock the rotation of the intake towards fuel in the neutral zone */
  private Command rotateDrivebaseToAngle(Rotation2d angle) {
    return DriveCommands.snapToAngle(drive, () -> angle, rotationToleranceDegrees);
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
                // Don't start shooting until you know you can actually score
                // in the hub
                Commands.waitUntil(
                        BobotState.isTooCloseToHubToScore()
                            .negate()
                            .and(shooter.isNearHubVelocity(RotationsPerSecond.of(2.0))))
                    .andThen(feeder.runFeederVoltage())),
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

  private Command stashedIntake() {
    return Commands.sequence(intake.endeffectorStashed().until(intake.isNearGoal(Degrees.of(8))))
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

  private Command BLineTrajectory(String name, boolean mirrorVertically) {
    Path path = new Path(name);

    // Mirror the Path vertically to handle both sides of the driver station POV
    if (mirrorVertically) {
      path.mirror();
    }

    return Commands.sequence(drive.getBLineBuilder().build(path))
        .beforeStarting(Commands.print(name + ": start"))
        .andThen(Commands.print(name + ": end"));
  }

  private Command BLineTrajectory(String name) {
    return BLineTrajectory(name, false);
  }

  private Command BLineTrajectorywithIntake(String name) {
    return Commands.deadline(
            drive.getBLineBuilder().build(new Path(name)), intake.runRollersTorqueCurrent())
        .beforeStarting(Commands.print(name + ": start"))
        .andThen(Commands.sequence(Commands.print(name + ": end")), intake.stop());
  }

  private Command BLineTrajectorywithIntakeLift(String name, double intaketime) {
    return this.BLineTrajectorywithIntakeLift(name, intaketime, false);
  }

  private Command BLineTrajectorywithIntakeLift(
      String name, double intaketime, boolean mirrorVertically) {
    Path path = new Path(name);

    // Mirror the Path vertically to handle both sides of the driver station POV
    if (mirrorVertically) {
      path.mirror();
    }

    FollowPath followCommand = (FollowPath) drive.getBLineBuilder().build(path);

    return Commands.deadline(
            followCommand,
            Commands.deadline(Commands.waitSeconds(intaketime), intake.runRollersTorqueCurrent())
                .andThen(intake.endeffectorLift()))
        .beforeStarting(Commands.print(name + ": start"))
        .andThen(Commands.sequence(Commands.print(name + ": end")), intake.stop());
  }

  /**
   * Calls {@link #BLineTrajectoryIntakeAndLiftAfterTranslationSegment(String, int, boolean)}
   * without mirroring the original BLine path
   */
  private Command BLineTrajectoryIntakeAndLiftAfterTranslationSegment(
      String name, int segmentNumber) {
    return this.BLineTrajectoryIntakeAndLiftAfterTranslationSegment(name, segmentNumber, false);
  }

  /**
   * Follows a BLine path with rollers active, but after {@code segmentNumber}, lift the intake
   *
   * <p>Use `segment number` as the index in BLine GUI for translation/waypoint markers. We may
   * multiply it by 2 because I think it's counting both X and Y?
   *
   * <p>This _should_ help get over the bump more consistently
   */
  private Command BLineTrajectoryIntakeAndLiftAfterTranslationSegment(
      String name, int segmentNumber, boolean mirrorVertically) {
    Path path = new Path(name);

    // Mirror the Path vertically to handle both sides of the driver station POV
    if (mirrorVertically) {
      path.mirror();
    }

    FollowPath followCommand = (FollowPath) drive.getBLineBuilder().build(path);

    return Commands.parallel(
        followCommand,
        intake
            .runRollersTorqueCurrent()
            .until(() -> followCommand.getCurrentTranslationElementIndex() >= (segmentNumber))
            .andThen(intake.endeffectorLift()));
  }

  private Command BLineTrajectoryWithHubRotation(String name) {
    return this.BLineTrajectoryWithHubRotation(name, false);
  }

  /** Follows {@code name} with hub rotation override */
  private Command BLineTrajectoryWithHubRotation(String name, boolean mirrorVertically) {
    Path path = new Path(name);

    // Mirror the Path vertically to handle both sides of the driver station POV
    if (mirrorVertically) {
      path.mirror();
    }

    return drive
        .followPathWithRotationTarget(path, () -> BobotState.getHubVARC().getRotationTarget())
        .beforeStarting(Commands.print(name + ": start"))
        .andThen(Commands.print(name + ": end"));
  }

  /** Driver Station Right Trench Auto */
  public Command blazikenTheMegaEvolution() {
    return blazikenMegaEvolutionImpl(false);
  }

  /** Driver Station Left Trench Auto */
  public Command swampertTheMegaEvolution() {
    return blazikenMegaEvolutionImpl(true);
  }

  public Command beedrillTheMegaEvolution() {
    return beedrill(false);
  }

  public Command slowpokeTheMegaEvolution() {
    return beedrill(true);
  }

  public Command evilDitto() {
    return evilDitto(false);
  }

  public Command dangerDitto() {
    return evilDitto(true);
  }

  public Command sableyeTheMegaEvolution() {
    return sableye(false);
  }

  public Command gengarTheMegaEvolution() {
    return sableye(true);
  }

  public Command diglett() {
    return diglett(false);
  }

  public Command gardevoirTheMegaEvolution() {
    return gardevoir(false);
  }

  private Command BIDOOF(boolean mirrorVertically) {
    return BLineTrajectory("Bidoof", mirrorVertically);
  }

  /**
   * Implementation of Blaziken and Swampert that flips based on which Trench it starts from
   *
   * @param mirrorVertically whether to flip the path over the vertical center of the field
   */
  private Command blazikenMegaEvolutionImpl(boolean mirrorVertically) {
    return Commands.sequence(
        intake.endeffectorDeploy(),
        Commands.waitUntil(intake.isNearGoal(Degrees.of(8.0))),
        BLineTrajectoryIntakeAndLiftAfterTranslationSegment("Blaziken-1", 14, mirrorVertically),
        intake.endeffectorDeploy(),
        Commands.deadline(
            Commands.waitTime(Seconds.of(4.0)),
            BLineTrajectoryWithHubRotation("Blaziken-2", mirrorVertically),
            this.handleInterpolateShooter("Shot 1", 6.0),
            intake.flickIntakeRepeatedly(
                Degrees.of(20.0),
                Degrees.of(35.0),
                Degrees.of(0.0),
                Degrees.of(5.0),
                Seconds.of(6.0))),
        // this.deployIntake(),
        // intake.runRollersTorqueCurrent(),
        this.feeder.stop(),
        this.intake.stop(),
        this.intake.stopPivot(),
        this.shooter.stop(),
        intake.endeffectorDeploy(),
        Commands.waitUntil(intake.isNearGoal(Degrees.of(8.0))),
        // this.rotateDrivebaseToAngle(Rotation2d.kCW_90deg),
        BLineTrajectoryIntakeAndLiftAfterTranslationSegment("Blaziken-3-Alt", 18, mirrorVertically),
        Commands.deadline(
            Commands.waitTime(Seconds.of(3.5)),
            this.rotateToHub(),
            // BLineTrajectoryWithHubRotation("Blaziken-4", mirrorVertically),
            this.handleInterpolateShooter("Shot 1", 5.5)));
    //     intake.runRollersTorqueCurrent()),
    // this.feeder.stop(),
    // intake.endeffectorDeploy(),
    // Commands.waitUntil(intake.isNearGoal(Degrees.of(8.0))),
    // BLineTrajectoryIntakeAndLiftAfterTranslationSegment("Blaziken-5", 18, mirrorVertically));
  }

  /**
   * Implementation of Blaziken and Swampert that flips based on which Trench it starts from
   *
   * @param mirrorVertically whether to flip the path over the vertical center of the field
   */
  private Command beedrill(boolean mirrorVertically) {
    return Commands.sequence(
        intake.endeffectorDeploy(),
        Commands.waitUntil(intake.isNearGoal(Degrees.of(8)))
            .onlyIf(() -> !this.shouldStartOnBump()),
        Commands.waitSeconds(1.0),
        this.BIDOOF(mirrorVertically).onlyIf(() -> this.shouldStartOnBump()),
        BLineTrajectoryIntakeAndLiftAfterTranslationSegment("Beedrill-1", 18, mirrorVertically),
        this.rotateToHub(),
        Commands.deadline(
            Commands.waitTime(Seconds.of(8.0)),
            this.handleInterpolateShooter("Shot 1", 8.0),
            Commands.waitSeconds(1.0)
                .andThen(
                    intake.flickIntakeRepeatedly(
                        Degrees.of(20.0),
                        Degrees.of(50.0),
                        Degrees.of(0.0),
                        Degrees.of(5.0),
                        Seconds.of(8.0)))));
  }

  /**
   * Implementation of Blaziken and Swampert that flips based on which Trench it starts from
   *
   * @param mirrorVertically whether to flip the path over the vertical center of the field
   */
  private Command evilDitto(boolean mirrorVertically) {
    return Commands.sequence(
        intake.endeffectorDeploy(),
        Commands.waitUntil(intake.isNearGoal(Degrees.of(8))),
        Commands.waitSeconds(0),
        BLineTrajectory("Evil-Ditto-1"),
        this.stashedIntake(),
        BLineTrajectory("Evil-Ditto-2"));
  }

  /**
   * Implementation of Blaziken and Swampert that flips based on which Trench it starts from
   *
   * @param mirrorVertically whether to flip the path over the vertical center of the field
   */
  private Command sableye(boolean mirrorVertically) {
    return Commands.sequence(
        intake.endeffectorDeploy(),
        Commands.waitUntil(intake.isNearGoal(Degrees.of(8))),
        Commands.waitSeconds(0),
        BLineTrajectorywithIntake("Sableye"));
  }

  /**
   * Implementation of Blaziken and Swampert that flips based on which Trench it starts from
   *
   * @param mirrorVertically whether to flip the path over the vertical center of the field
   */
  private Command diglett(boolean mirrorVertically) {
    return Commands.sequence(
        intake.endeffectorDeploy(),
        Commands.waitUntil(intake.isNearGoal(Degrees.of(8))),
        Commands.waitSeconds(0),
        BLineTrajectorywithIntake("Diglett"),
        this.rotateToHub(),
        Commands.deadline(
            Commands.waitTime(Seconds.of(8.0)),
            this.handleInterpolateShooter("Shot 1", 8.0),
            Commands.waitSeconds(1.0)
                .andThen(
                    intake.flickIntakeRepeatedly(
                        Degrees.of(20.0),
                        Degrees.of(50.0),
                        Degrees.of(0.0),
                        Degrees.of(5.0),
                        Seconds.of(8.0)))));
  }

  /**
   * Implementation of Blaziken and Swampert that flips based on which Trench it starts from
   *
   * @param mirrorVertically whether to flip the path over the vertical center of the field
   */
  private Command gardevoir(boolean mirrorVertically) {
    return Commands.sequence(
        intake.endeffectorDeploy(),
        Commands.waitUntil(intake.isNearGoal(Degrees.of(8))),
        // Commands.print("Hey Peyton start on Bump")
        //     .onlyIf(() -> SmartDashboard.getBoolean(shouldStartOnBumpKey, false)),
        Commands.waitSeconds(0),
        BLineTrajectory("Gardevoir"),
        Commands.waitSeconds(2.0),
        BLineTrajectory("Gardevoir-2"),
        this.rotateToHub(),
        Commands.deadline(
            Commands.waitTime(Seconds.of(8.0)),
            this.handleInterpolateShooter("Shot 1", 8.0),
            Commands.waitSeconds(1.0)
                .andThen(
                    intake.flickIntakeRepeatedly(
                        Degrees.of(20.0),
                        Degrees.of(50.0),
                        Degrees.of(0.0),
                        Degrees.of(5.0),
                        Seconds.of(8.0)))));
  }

  public Command bline1ft() {
    return Commands.sequence(BLineTrajectory("1ft-bline"));
  }

  public Command bline2ft() {
    return Commands.sequence(BLineTrajectory("2ft-bline"));
  }
}
