package frc.robot.subsystems.prototyping;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.LoggedTunableNumber;
import org.robotzgarage.frc2026.subsystems.flywheel.Flywheel;
import org.robotzgarage.frc2026.subsystems.rollers.follow.FollowRollers;
import org.robotzgarage.frc2026.subsystems.rollers.follow.FollowRollersIO;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;

public class ShooterV1 extends SubsystemBase {
  private final String name = "ShooterJan28/";

  private static final LoggedTunableNumber shooterAmps =
      new LoggedTunableNumber("ShooterJan28/ShooterAmps", 20.0);
  private static final LoggedTunableNumber shooterVoltage =
      new LoggedTunableNumber("ShooterJan28/ShooterVoltage", 9.75);
  private static final LoggedTunableNumber shooterPercent =
      new LoggedTunableNumber("ShooterJan28/ShooterPercent", 0.78);
  private static final LoggedTunableNumber shooterVelocity =
      new LoggedTunableNumber("ShooterJan28/ShooterVelocity", 67.0);
  private static final LoggedTunableNumber shooterkp =
      new LoggedTunableNumber("ShooterJan28/ShooterkP", 3.0);

  private static final LoggedTunableNumber towerAmps =
      new LoggedTunableNumber("ShooterJan28/FeederAmps", 0.0);
  private static final LoggedTunableNumber towerVoltage =
      new LoggedTunableNumber("ShooterJan28/FeederVoltage", 10);
  private static final LoggedTunableNumber towerPercent =
      new LoggedTunableNumber("ShooterJan28/FeederPercent", 0.5);

  private static final LoggedTunableNumber kickerAmps =
      new LoggedTunableNumber("ShooterJan28/KickerAmps", 0.0);
  private static final LoggedTunableNumber kickerVoltage =
      new LoggedTunableNumber("ShooterJan28/KickerVoltage", 0.0);
  private static final LoggedTunableNumber kickerPercent =
      new LoggedTunableNumber("ShooterJan28/KickerPercent", 0.0);

  // Subsystems
  private final FollowRollers shooterFlyWheels;
  private final Flywheel tower;
  private final Flywheel kicker;

  // Constructor to decide if it's real or sim
  public ShooterV1(
      FollowRollersIO shooterFlyWheelIO, SingleRollerIO towerIO, SingleRollerIO kickerIO) {
    this.shooterFlyWheels = new FollowRollers(name + "ShooterFlywheel", shooterFlyWheelIO);
    tower = new Flywheel(name + "Tower/", towerIO, ShooterV1Constants.towerFlywheelConfig);
    kicker = new Flywheel(name + "Kicker/", kickerIO, ShooterV1Constants.kickerFlywheelConfig);
  }

  public void periodic() {
    // Periodic modes
    tower.periodic();
    kicker.periodic();
    shooterFlyWheels.periodic();

    if (shooterkp.hasChanged(hashCode())) {
      this.shooterFlyWheels.updatePID(shooterkp.get(), 0, 0);
    }
  }

  public Command runShooterTuning() {
    return run(
        () -> {
          shooterFlyWheels.setVelocityTorqueCurrent(RotationsPerSecond.of(shooterVelocity.get()));
          tower.runVolts(towerVoltage.get());
          kicker.runVolts(towerVoltage.get());
        });
  }

  public Command runShooterTorqueCurrent() {
    return run(
        () -> {
          double clampedCurrent = MathUtil.clamp(shooterAmps.get(), -100.0, 100.0);
          shooterFlyWheels.runTorqueCurrent(Amps.of(clampedCurrent));
        });
  }

  public Command runShooterVoltage() {
    return run(
        () -> {
          double clampedVoltage = MathUtil.clamp(shooterVoltage.get(), -12.0, 12.0);
          shooterFlyWheels.runVolts(clampedVoltage);
        });
  }

  public Command runShooterPercentOut() {
    return run(
        () -> {
          double clampedPercent = MathUtil.clamp(shooterPercent.get(), -1.0, 1.0);
          shooterFlyWheels.runPercentOutput(clampedPercent);
        });
  }

  public Command runShooterVelocityTorqueCurrent() {
    return run(
        () -> {
          double clampedVelocity = MathUtil.clamp(shooterVelocity.get(), 0.0, 200.0);
          shooterFlyWheels.setVelocityTorqueCurrent(RotationsPerSecond.of(clampedVelocity));
        });
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

  public Command stop() {
    return run(
        () -> {
          shooterFlyWheels.stop();
          tower.stop();
          kicker.stop();
        });
  }
}
