package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.LoggedTunableNumber;
import org.robotzgarage.frc2026.subsystems.flywheel.FlywheelFollower;
import org.robotzgarage.frc2026.subsystems.rollers.follow.FollowRollersIO;

public class Shooter extends SubsystemBase {
  private static final String name = "/Shooter";

  private static final LoggedTunableNumber shooterAmps =
      new LoggedTunableNumber(name + "/ShooterAmps", 20.0);
  private static final LoggedTunableNumber shooterVoltage =
      new LoggedTunableNumber(name + "/ShooterVoltage", 9.75);
  private static final LoggedTunableNumber shooterPercent =
      new LoggedTunableNumber(name + "/ShooterPercent", 0.78);
  private static final LoggedTunableNumber shooterVelocity =
      new LoggedTunableNumber(name + "/ShooterVelocity", 67.0);

  // Subsystems
  private final FlywheelFollower shooter;

  // Constructor to decide if it's real or sim
  public Shooter(FollowRollersIO shooterFlyWheelIO) {
    this.shooter =
        new FlywheelFollower(
            name + "/Shooter", shooterFlyWheelIO, ShooterConstants.shooterFlywheelConfig);
  }

  public void periodic() {
    // Periodic modes
    shooter.periodic();
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
        });
  }
}
