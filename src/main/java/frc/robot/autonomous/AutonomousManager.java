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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.prototyping.IntakeV1;
import frc.robot.subsystems.prototyping.ShooterV1;

public class AutonomousManager {
  private final Drive drive;
  private final IntakeV1 intake;
  private final ShooterV1 shooter;

  private static final double rotationToleranceDegrees = 1.0;
  private static final Time MAX_TIMEOUT = Seconds.of(20);

  public AutonomousManager(Drive drive, IntakeV1 intakeV1, ShooterV1 shooterV1) {
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

  private Command rotateToDropOff() {
    return DriveCommands.snapToAngle(
        drive, () -> BobotState.getClosestDropoff().getRotationTarget(), rotationToleranceDegrees);
  }

  private Command handleShot(String prefix, double velocity, double tolerance, double emptyTime) {
    String formattedPrefix = prefix + " - ";
    AngularVelocity shooterVelocity = RotationsPerSecond.of(velocity);
    AngularVelocity shooterTolerance = RotationsPerSecond.of(tolerance);
    Time estimatedShooterEmptyTime = Seconds.of(emptyTime);

    return Commands.sequence(
        Commands.print(formattedPrefix + "Starting Shooter Flywheel"),
        shooter
            .runShooterVelocityTorqueCurrent(shooterVelocity)
            .until(shooter.isShooterFlywheelNearRPS(shooterVelocity, shooterTolerance))
            .andThen(Commands.print(formattedPrefix + "Flywheel has met target velocity")),
        Commands.sequence(
            Commands.print(formattedPrefix + "Shooter is shooting, feeder and tower are running")),
        shooter.runShooterReal(shooterVelocity).withTimeout(estimatedShooterEmptyTime),
        shooter.stop(),
        Commands.print(formattedPrefix + "Finished shooting"));
  }

  /** deploys intake to set position to intake at */
  private Command deployIntake() {
    return Commands.sequence(
        intake.endeffectorGoal().until(intake.isNearGoal(Degrees.of(5))),
        Commands.print("Deploy Intake"));
  }

  /** retracks intake to starting position */
  private Command retractIntake() {
    return Commands.sequence(
        intake.endeffectorHome().until(intake.isNearGoal(Degrees.of(5))),
        Commands.print("Retract Intake"));
  }

  public Command BLineMechcanismTest() {
    return Commands.sequence(
        deployIntake(),
        drive.BLineTrajectory("BLineTest-01"),
        retractIntake(),
        this.rotateToHub(),
        handleShot("Test shot", 50.0, 1.0, 2.0),
        drive.BLineTrajectory("BLineTest-02"),
        this.rotateToHub(),
        handleShot("Test shot 2", 80.0, 1.0, 1.0)
        );
  }

  // public Command gastly() {
  //   return Commands.sequence(
  //       BLineTrajectory("Gastly-01"),
  //       this.deployIntake(),
  //       BLineTrajectory("Gastly-02"),
  //       BLineTrajectory("Gastly-03"),
  //       BLineTrajectory("Gastly-04"),
  //       BLineTrajectory("Gastly-05"),
  //       this.rotateToHub(),
  //       this.handleShot(80.0, 1.0, 2.0),
  //       this.deployIntake(),
  //       BLineTrajectory("Gastly-06"),
  //       BLineTrajectory("Gastly-07"),
  //       this.rotateToHub(),
  //       this.handleShot(80.0, 1.0, 2.0));
  // }

  // public Command falinks() {

  //   Path crossTheBumpThenRotate = new Path("Falinks - 01");
  //   Path driveIntoFuelFirst = new Path("Falinks - 02");
  //   Path driveBackIntoFuel = new Path("Falinks - 02A");
  //   Path crossTheBumpToAlliance = new Path("Falinks - 03");

  //   return Commands.sequence(
  //           this.rotateToHub(),
  //           this.handleShot("First Shot"),
  //           Commands.sequence(
  //               Commands.print("Driving across the Bump"),
  //               drive.getBLineBuilder().build(crossTheBumpThenRotate)),
  //           this.deployIntake(),
  //           Commands.sequence(
  //               Commands.print("Backing up into Fuel"),
  //               Commands.race(
  //                       Commands.sequence(
  //                           drive.getBLineBuilder().build(driveIntoFuelFirst),
  //                           drive.getBLineBuilder().build(driveBackIntoFuel)),
  //                       intake.runIntakePercentOut(0.75))
  //                   .andThen(intake.stop())),
  //           Commands.sequence(
  //               Commands.print("Driving back over bump"),
  //               drive.getBLineBuilder().build(crossTheBumpToAlliance)),
  //           Commands.sequence(Commands.print("Rotate to Hub"), this.rotateToHub()),
  //           Commands.sequence(
  //               Commands.print("Shoot again"), this.handleShot("Second Shot"), shooter.stop()))

  //       // .withTimeout(MAX_TIMEOUT)
  //       .andThen(Commands.print("Falinks - End"))
  //       .beforeStarting(Commands.print("Falinks - Start"));
  // }
}
