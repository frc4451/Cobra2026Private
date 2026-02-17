package frc.robot.subsystems.prototyping;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.LoggedTunableNumber;
import org.robotzgarage.frc2026.subsystems.flywheel.Flywheel;
import org.robotzgarage.frc2026.subsystems.rollers.follow.FollowRollers;
import org.robotzgarage.frc2026.subsystems.rollers.follow.FollowRollersIO;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;

public class ShooterV1 extends SubsystemBase {
  private static final String name = "/ShooterV1";

  private static final LoggedTunableNumber shooterAmps =
      new LoggedTunableNumber(name + "/ShooterAmps", 20.0);
  private static final LoggedTunableNumber shooterVoltage =
      new LoggedTunableNumber(name + "/ShooterVoltage", 9.75);
  private static final LoggedTunableNumber shooterPercent =
      new LoggedTunableNumber(name + "/ShooterPercent", 0.78);
  private static final LoggedTunableNumber shooterVelocity =
      new LoggedTunableNumber(name + "/ShooterVelocity", 67.0);

  private static final LoggedTunableNumber towerAmps =
      new LoggedTunableNumber(name + "/FeederAmps", 20.0);
  private static final LoggedTunableNumber towerVoltage =
      new LoggedTunableNumber(name + "/FeederVoltage", 10);
  private static final LoggedTunableNumber towerPercent =
      new LoggedTunableNumber(name + "/FeederPercent", 0.5);

  private static final LoggedTunableNumber kickerAmps =
      new LoggedTunableNumber(name + "/KickerAmps", 20.0);
  private static final LoggedTunableNumber kickerVoltage =
      new LoggedTunableNumber(name + "/KickerVoltage", 7.0);
  private static final LoggedTunableNumber kickerPercent =
      new LoggedTunableNumber(name + "/KickerPercent", 0.5);

  // Subsystems
  private final FollowRollers shooter;
  private final Flywheel tower;
  private final Flywheel kicker;

  // Constructor to decide if it's real or sim
  public ShooterV1(
      FollowRollersIO shooterFlyWheelIO, SingleRollerIO towerIO, SingleRollerIO kickerIO) {
    this.shooter = new FollowRollers(name + "/Shooter", shooterFlyWheelIO);
    tower = new Flywheel(name + "/Tower", towerIO, ShooterV1Constants.towerFlywheelConfig);
    kicker = new Flywheel(name + "/Kicker", kickerIO, ShooterV1Constants.kickerFlywheelConfig);
  }

  public void periodic() {
    // Periodic modes
    tower.periodic();
    kicker.periodic();
    shooter.periodic();
  }

  public Command runShooterTuning() {
    return run(
        () -> {
          shooter.setVelocityTorqueCurrent(RotationsPerSecond.of(shooterVelocity.get()));
          tower.runVolts(towerVoltage.get());
          kicker.runVolts(towerVoltage.get());
        });
  }

  public Command runShooterReal(AngularVelocity targetShooterVelocity) {
    return run(
        () -> {
          shooter.setVelocityTorqueCurrent(targetShooterVelocity);
          // These two are guesses from the LoggedTunableNumbers that Josh gave us
          tower.runVolts(10);
          kicker.runVolts(7.0);
        });
  }

  public Command runShooterTorqueCurrent() {
    return run(
        () -> {
          double clampedCurrent = MathUtil.clamp(shooterAmps.get(), -100.0, 100.0);
          shooter.runTorqueCurrent(Amps.of(clampedCurrent));
        });
  }

  public Command runShooterVoltage() {
    return run(
        () -> {
          double clampedVoltage = MathUtil.clamp(shooterVoltage.get(), -12.0, 12.0);
          shooter.runVolts(clampedVoltage);
        });
  }

  public Command runShooterPercentOut() {
    return run(
        () -> {
          double clampedPercent = MathUtil.clamp(shooterPercent.get(), -1.0, 1.0);
          shooter.runPercentOutput(clampedPercent);
        });
  }

  public Command runShooterVelocityTorqueCurrent() {
    return run(
        () -> {
          double clampedVelocity = MathUtil.clamp(shooterVelocity.get(), 0.0, 200.0);
          shooter.setVelocityTorqueCurrent(RotationsPerSecond.of(clampedVelocity));
        });
  }

  public Command runShooterVelocityTorqueCurrent(AngularVelocity velocity) {
    return run(() -> shooter.setVelocityTorqueCurrent(velocity));
  }

  public Command runTowerTorqueCurrent() {
    return run(
        () -> {
          double clampedCurrent = MathUtil.clamp(towerAmps.get(), -100.0, 100.0);
          tower.runTorqueCurrent(Amps.of(clampedCurrent));
        });
  }

  public Command runTowerVoltage() {
    return run(
        () -> {
          double clampedVoltage = MathUtil.clamp(towerVoltage.get(), -12.0, 12.0);
          tower.runVolts(clampedVoltage);
        });
  }

  public Command runTowerPercentOut() {
    return run(
        () -> {
          double clampedPercent = MathUtil.clamp(towerPercent.get(), -1.0, 1.0);
          tower.runPercentOutput(clampedPercent);
        });
  }

  public Command runKickerTorqueCurrent() {
    return run(
        () -> {
          double clampedCurrent = MathUtil.clamp(kickerAmps.get(), -100.0, 100.0);
          kicker.runTorqueCurrent(Amps.of(clampedCurrent));
        });
  }

  public Command runKickerVoltage() {
    return run(
        () -> {
          double clampedVoltage = MathUtil.clamp(kickerVoltage.get(), -12.0, 12.0);
          kicker.runVolts(clampedVoltage);
        });
  }

  public Command runKickerPercentOut() {
    return run(
        () -> {
          double clampedPercent = MathUtil.clamp(kickerPercent.get(), -1.0, 1.0);
          kicker.runPercentOutput(clampedPercent);
        });
  }

  /** You need to know when the Flywheel is at the velocity target we want */
  public Trigger isShooterFlywheelNearRPS(
      AngularVelocity taretVelocity, AngularVelocity tolerance) {
    return new Trigger(
        () -> this.shooter.getInputs().leaderVelocity.isNear(taretVelocity, tolerance));
  }

  public Command stop() {
    return runOnce(
        () -> {
          shooter.stop();
          tower.stop();
          kicker.stop();
        });
  }

  public Command stopFeeding() {
    return runOnce(
        () -> {
          tower.stop();
          kicker.stop();
        });
  }
}
