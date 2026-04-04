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
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.robotzgarage.frc2026.math.interpolation.ShootingInterpolator.InterpolatedCalculation;

public class AutonomousManager {
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Feeder feeder;

  private static final double rotationToleranceDegrees = 3.0;
  private static final Time MAX_TIMEOUT = Seconds.of(20);

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

  /**
   * Full method to use to handle the shooting for the shooter
   *
   * <p>
   *
   * @param prefix uses a {@code String} to tie the steps of {@code handleShot} for easier reading
   *     of the console in {@code AdvantageScope}
   * @param velocity uses a {@code double} that goes into an {@code AngularVelocity(RPS)} for
   *     shooter controll
   * @param tolerance uses a {@code double} that goes into an {@code AngularVelocity(RPS)} to tell
   *     the shooter to start shooting
   * @param emptyTime uses a {@code double} that goes into a {@code TimeUnit(Seconds)} for the
   *     estimated hopper empty time / when it stops the shooter
   */
  private Command handleShot(String prefix, double velocity, double tolerance, double emptyTime) {
    String formattedPrefix = prefix + ": ";
    AngularVelocity shooterVelocity = RotationsPerSecond.of(velocity);

    return Commands.sequence(
            Commands.print(formattedPrefix + "Starting Shooter Flywheel"),
            shooter
                // set velocity to shooter at with torque current
                .runShooterVelocityTorqueCurrent(shooterVelocity)
                .until(
                    // wait to start tower, until shooter is at velocity
                    shooter.isShooterFlywheelNearRPS(
                        shooterVelocity, RotationsPerSecond.of(tolerance))),
            // shoots shooter for the estimated empty time
            Commands.print(formattedPrefix + "start feeder"),
            feeder.runFeederVoltage(),
            Commands.print(formattedPrefix + "start shooter"),
            shooter
                .runShooterVelocityTorqueCurrent(shooterVelocity)
                .withTimeout(Seconds.of(emptyTime)),
            shooter.stop(),
            feeder.stop())
        .beforeStarting(Commands.print(formattedPrefix + "start"))
        .andThen(Commands.print(formattedPrefix + "end"));
  }

  /**
   * Full method to use to handle the shooting for the shooter
   *
   * <p>
   *
   * @param prefix Uses a {@code String} to tie the steps of {@code handleShot} for easier reading
   *     of the console in {@code AdvantageScope}
   * @param velocity Uses a {@code double} that goes into an {@code AngularVelocity(RPS)} for
   *     shooter controll
   * @param emptyTime Uses a {@code double} that goes into a {@code TimeUnit(Seconds)} for the
   *     estimated hopper empty time / when it stops the shooter @DefaultParameter Default Tolerance
   *     of 1 RPS. Tolerance uses a {@code double} that goes into an {@code AngularVelocity(RPS)} to
   *     tell the shooter to start shooting
   */
  private Command handleShot(String prefix, double velocity, double emptyTime) {
    String formattedPrefix = prefix + ": ";
    AngularVelocity shooterVelocity = RotationsPerSecond.of(velocity);

    return Commands.sequence(
            Commands.print(formattedPrefix + "Starting Shooter Flywheel"),
            shooter
                // set velocity to shooter at with torque current
                .runShooterVelocityTorqueCurrent(shooterVelocity)
                .until(
                    // wait to start tower, until shooter is at velocity
                    shooter.isShooterFlywheelNearRPS(shooterVelocity, RotationsPerSecond.of(1.0))),
            // shoots shooter for the estimated empty time
            Commands.print(formattedPrefix + "start shooter"),
            shooter
                .runShooterVelocityTorqueCurrent(shooterVelocity)
                .withTimeout(Seconds.of(emptyTime)),
            shooter.stop())
        .beforeStarting(Commands.print(formattedPrefix + "start"))
        .andThen(Commands.print(formattedPrefix + "end"));
  }

  /**
   * Full method to use to handle the shooting for the shooter
   *
   * <p>
   *
   * @param prefix Uses a {@code String} to tie the steps of {@code handleShot} for easier reading
   *     of the console in {@code AdvantageScope}
   * @param emptyTime Uses a {@code double} that goes into a {@code TimeUnit(Seconds)} for the
   *     estimated hopper empty time / when it stops the shooter
   */
  private Command handleShotInterpolation(String prefix, double emptyTime, double squeezeTime) {
    String formattedPrefix = prefix + ": ";
    InterpolatedCalculation calculation = BobotState.getHubInterpolator().getCalculation();
    AngularVelocity velocity = calculation.primaryVelocity();

    return Commands.sequence(
            // Commands.print(formattedPrefix + "Starting Shooter Flywheel. Velocity: " +
            // velocity.baseUnitMagnitude()),
            Commands.print(formattedPrefix + "Starting Shooter Flywheel"),
            shooter
                // set velocity to shooter at with torque current
                .runShooterVelocityTorqueCurrent(velocity)
                .until(
                    // wait to start tower, until shooter is at velocity
                    shooter.isShooterFlywheelNearRPS(velocity, RotationsPerSecond.of(1.0))),
            // shoots shooter for the estimated empty time
            Commands.print(formattedPrefix + "start shooter"),
            Commands.deadline(
                shooter.runShooterToHub().withTimeout(Seconds.of(emptyTime)),
                feeder.runFeederVoltage(),
                Commands.waitSeconds(squeezeTime).andThen(intake.wiggelsqueeze())),
            feeder.stop(),
            shooter.stop())
        .beforeStarting(Commands.print(formattedPrefix + "start"))
        .andThen(Commands.print(formattedPrefix + "end"));
  }

  private Command handleShotInterpolation2(String prefix, double emptyTime, double squeezeTime) {
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

  // private Command handleShotInterpolation3(String prefix, double emptyTime, double squeezeTime) {
  //   String formattedPrefix = prefix + ": ";

  //   return Commands.sequence(
  //           Commands.print(formattedPrefix + "start shooter"),
  //           Commands.parallel(
  //               Commands.deadline(
  //                   Commands.waitTime(Seconds.of(emptyTime)),
  //                   shooter.runShooterToHub(),
  //                   feeder.runFeederVoltage()),
  //               Commands.sequence(
  //                   Commands.waitTime(Seconds.of(squeezeTime)), intake.wiggelsqueeze())),
  //           feeder.stop(),
  //           shooter.stop())
  //       .beforeStarting(Commands.print(formattedPrefix + "start"))
  //       .andThen(Commands.print(formattedPrefix + "end"));
  // }

  /** deploys intake to set position to intake at */
  private Command deployIntake() {
    return Commands.sequence(intake.endeffectorDeploy().until(intake.isNearGoal(Degrees.of(3))))
        .beforeStarting(Commands.print("deploy: start"))
        .andThen(Commands.print("deploy: end"));
  }

  /** retracks intake to starting position */
  private Command retrackIntake() {
    return Commands.sequence(intake.endeffectorStart().until(intake.isNearGoal(Degrees.of(5))))
        .beforeStarting(Commands.print("retrack: start"))
        .andThen(Commands.print("retrack: end"));
  }

  /** lifts intake to an offset position for bump transversal */
  private Command liftIntake() {
    return Commands.sequence(intake.endeffectorLift())
        .until(intake.isNearGoal(Degrees.of(5)))
        .beforeStarting(Commands.print("lift: start"))
        .andThen(Commands.print("lift: end"));
  }

  /** Sqeeze intake for shooting */
  private Command squeezeIntake() {
    return Commands.sequence(intake.endeffectorSqueeze())
        .until(intake.isNearGoal(Degrees.of(5)))
        .beforeStarting(Commands.print("Squeeze: start"))
        .andThen(Commands.print("Squeeze: end"));
  }

  private Command wiggleWiggleWiggle() {
    return Commands.sequence(squeezeIntake(), Commands.waitSeconds(1.0), deployIntake());
  }

  private Command runIntake() {
    return intake.runRollersVoltageOut().beforeStarting(Commands.print("Intake rollers: start"));
  }

  private Command stopIntake() {
    return Commands.runOnce(() -> intake.stop())
        .beforeStarting(Commands.print("Intake rollers: stop"));
  }

  // private Command strafeShoot(String name) {
  //   return Commands.run(
  //           () ->
  //               new ShootOnTheMoveCommand(
  //                   drive,
  //                   () ->
  //                       (drive.getChassisSpeeds().vyMetersPerSecond
  //                           / drive.getMaxLinearSpeedMetersPerSec()),
  //                   () ->
  //                       (drive.getChassisSpeeds().vxMetersPerSecond
  //                           / drive.getMaxLinearSpeedMetersPerSec()),
  //                   () -> BobotState.getShooterAlignmentVARC(),
  //                   () -> BobotState.getShooterInterpolator()))
  //       .beforeStarting(Commands.print(name + ": Start Strafe Shoot"))
  //       .andThen(Commands.print(name + ": End Strafe Shoot"));
  // }

  // private Command BLineStrafe(String name) {
  //   return Commands.sequence(
  //     drive.getBLineBuilder().build(new Path(name)),
  //               new ShootOnTheMoveCommand(
  //                   drive,
  //                   () ->
  //                       (drive.getChassisSpeeds().vyMetersPerSecond
  //                           / drive.getMaxLinearSpeedMetersPerSec()),
  //                   () ->
  //                       (drive.getChassisSpeeds().vxMetersPerSecond
  //                           / drive.getMaxLinearSpeedMetersPerSec()),
  //                   () -> BobotState.getShooterAlignmentVARC(),
  //                   () -> BobotState.getShooterInterpolator()))
  //       .beforeStarting(Commands.print(name + ": Start Strafe Shoot"))
  //       .andThen(Commands.print(name + ": End Strafe Shoot"));
  // }

  private Command BLineTrajectory(String name) {
    return Commands.sequence(drive.getBLineBuilder().build(new Path(name)))
        .beforeStarting(Commands.print(name + ": start"))
        .andThen(Commands.print(name + ": end"));
  }

  private Command BLineTrajectorywithIntake(String name) {
    return Commands.deadline(
            drive.getBLineBuilder().build(new Path(name)), intake.runRollersVoltageOut())
        .beforeStarting(Commands.print(name + ": start"))
        .andThen(Commands.sequence(Commands.print(name + ": end")), intake.stop());
  }

  private Command BLineTrajectorywithLift(String name, double intaketime) {
    return Commands.deadline(
            drive.getBLineBuilder().build(new Path(name)),
            Commands.deadline(Commands.waitSeconds(intaketime), intake.runRollersVoltageOut())
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

  /**
   * Follows {@code name} with hub rotation override, with rotation applied only after we're within
   * threshold to start
   */
  private Command BLineTrajectoryWithHubRotation(String name, double hubRotationThresholdMeters) {
    return drive
        .followPathWithRotationTarget(
            new Path(name),
            () -> BobotState.getHubVARC().getRotationTarget(),
            hubRotationThresholdMeters)
        .beforeStarting(Commands.print(name + ": start"))
        .andThen(Commands.print(name + ": end"));
  }

  public Command himithy() {
    return Commands.sequence(
            this.deployIntake(),
            BLineTrajectory("OGHimithy-1"),
            BLineTrajectorywithIntake("OGHimithy-2"),
            this.liftIntake(),
            BLineTrajectory("OGHimithy-3"),
            this.rotateToHub(),
            this.handleShotInterpolation("shot 1", 5.0, 2.0))
        .beforeStarting(Commands.print("Path start"))
        .andThen(Commands.print("Path end"));
  }

  public Command himithyy() {
    return Commands.sequence(
            this.deployIntake(),
            BLineTrajectory("OGHimithy-1"),
            BLineTrajectorywithIntake("OGHimithyy-2"),
            this.liftIntake(),
            BLineTrajectory("OGHimithyy-3"),
            this.rotateToHub(),
            this.handleShotInterpolation("shot 1", 10.0, 2.0))
        .beforeStarting(Commands.print("Path start"))
        .andThen(Commands.print("Path end"));
  }

  public Command timithy() {
    return Commands.sequence(
            this.deployIntake(),
            BLineTrajectory("OGTimithy-1"),
            BLineTrajectorywithIntake("OGTimithy-2"),
            this.liftIntake(),
            BLineTrajectory("OGTimithy-3"),
            this.rotateToHub(),
            this.handleShotInterpolation("shot 1", 8.0, 3.0))
        .beforeStarting(Commands.print("Path start"))
        .andThen(Commands.print("Path end"));
  }

  public Command timithyy() {
    return Commands.sequence(
            this.deployIntake(),
            BLineTrajectory("OGTimithy-1"),
            BLineTrajectorywithIntake("OGTimithyy-2"),
            this.liftIntake(),
            BLineTrajectory("OGTimithyy-3"),
            this.rotateToHub(),
            this.handleShotInterpolation("shot 1", 10.0, 6.0))
        .beforeStarting(Commands.print("Path start"))
        .andThen(Commands.print("Path end"));
  }

  public Command timithy_1() {
    return Commands.sequence(
            this.deployIntake(),
            Commands.deadline(
                    BLineTrajectory("timithy-1"),
                    Commands.waitSeconds(1)
                        .andThen(
                            Commands.deadline(
                                Commands.waitSeconds(5), intake.runRollersVoltageOut())))
                .andThen(Commands.waitSeconds(1).andThen(intake.stop())),
            this.rotateToHub(),
            this.handleShotInterpolation("shot 1", 10.0, 6.0))
        .beforeStarting(Commands.print("Path start"))
        .andThen(Commands.print("Path end"));
  }

  public Command himithyTake1() {
    return Commands.sequence(
            this.deployIntake(),
            BLineTrajectorywithIntake("Himithy-1"),
            this.rotateToHub(),
            this.handleShotInterpolation("shot 1", 10.0, 6.0))
        .beforeStarting(Commands.print("Path start"))
        .andThen(Commands.print("Path end"));
  }

  public Command himithyTake2() {
    return Commands.sequence(
            Commands.sequence(
                this.deployIntake(),
                BLineTrajectorywithLift("Himithy-1", 5.0),
                this.rotateToHub(),
                this.handleShotInterpolation2("shot 1", 4.0, 2.0)),
            this.deployIntake(),
            BLineTrajectorywithIntake("Himithy-2"),
            this.rotateToHub(),
            this.handleShotInterpolation2("shot 2", 4.0, 2.0))
        .beforeStarting(Commands.print("Path start"))
        .andThen(Commands.print("Path end"));
  }

  // public Command eanDemo() {
  //   return Commands.sequence(
  //           this.deployIntake(),
  //           Commands.waitSeconds(1).until(intake.isNearGoal(Degrees.of(8)),
  //           BLineTrajectory("eanDemo-1"),
  //           this.rotateToHub(),
  //           this.handleShotInterpolation2("shot 1", 4.0, 2.0),
  //           this.deployIntake(),
  //           BLineTrajectoryWithHubRotation("eanDemo-2"),
  //           this.rotateToHub(),
  //           this.handleShotInterpolation2("shot 2", 4.0, 2.0))
  //       .beforeStarting(Commands.print("Path start"))
  //       .andThen(Commands.print("Path end")));
  // }

  public Command dimithy_1() {
    return Commands.sequence(
            this.deployIntake(),
            BLineTrajectory("dimithy-1"),
            Commands.waitSeconds(4),
            BLineTrajectory("dimithy-2"),
            this.rotateToHub(),
            this.handleShotInterpolation("shot 1", 10.0, 5.0))
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
            BLineTrajectorywithLift("Bimithy-1-SOTM", 5.0),
            Commands.deadline(
                Commands.waitTime(Seconds.of(4.0)),
                BLineTrajectoryWithHubRotation("Bimithy-2-SOTM"),
                this.handleInterpolateShooter("shot 1", 4.0),
                this.runIntake()),
            this.deployIntake(),
            BLineTrajectorywithIntake("Bimithy-3-SOTM"),
            BLineTrajectorywithIntake("Bimithy-3"),
            this.rotateToHub(),
            this.handleShotInterpolation2("shot 2", 4.0, 2.0))
        .beforeStarting(Commands.print("Path start"))
        .andThen(Commands.print("Path end"));
  }

  public Command bimithySmiley() {
    return Commands.sequence(
            Commands.sequence(
                this.deployIntake(),
                BLineTrajectorywithLift("Bimithy-1", 5.0),
                this.rotateToHub(),
                this.handleShotInterpolation2("shot 1", 4.0, 2.0)),
            this.deployIntake(),
            BLineTrajectorywithIntake("Bimithy-2"),
            BLineTrajectorywithIntake("Bimithy-3"),
            this.rotateToHub(),
            this.handleShotInterpolation2("shot 2", 4.0, 2.0))
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
  // public Command strafeShot() {
  //   return Commands.sequence(
  //       Commands.deadline(BLineTrajectory("Strafe_Shoot"), strafeShoot("strafe 1")));
  // }
}
