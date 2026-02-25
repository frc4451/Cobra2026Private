package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.LoggedTunableNumber;
import org.robotzgarage.frc2026.subsystems.flywheel.Flywheel;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;
import org.robotzgarage.frc2026.util.TuningUtils.TuningUnit;

public class Feeder extends SubsystemBase {
  private static final String name = "/Feeder";

  private static final LoggedTunableNumber towerAmps =
      new LoggedTunableNumber(name + "/TowerAmps", 20.0);
  private static final LoggedTunableNumber towerVoltage =
      new LoggedTunableNumber(name + "/TowerVoltage", 10);
  private static final LoggedTunableNumber towerPercent =
      new LoggedTunableNumber(name + "/TowerPercent", 0.5);

  private static final LoggedTunableNumber kickerAmps =
      new LoggedTunableNumber(name + "/KickerAmps", 20.0);
  private static final LoggedTunableNumber kickerVoltage =
      new LoggedTunableNumber(name + "/KickerVoltage", 7.0);
  private static final LoggedTunableNumber kickerPercent =
      new LoggedTunableNumber(name + "/KickerPercent", 0.5);

  // Subsystems
  private final Flywheel tower;
  private final Flywheel kicker;

  // Constructor to decide if it's real or sim
  public Feeder(SingleRollerIO towerIO, SingleRollerIO kickerIO) {
    tower = new Flywheel(name + "/Tower", towerIO, FeederConstants.towerFlywheelConfig);
    kicker = new Flywheel(name + "/Kicker", kickerIO, FeederConstants.kickerFlywheelConfig);
  }

  public void periodic() {
    // Periodic modes
    tower.periodic();
    kicker.periodic();
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
    return runOnce(
        () -> {
          tower.stop();
          kicker.stop();
        });
  }

  public Command towerFeedForwardCharacterization(TuningUnit unit) {
    return this.tower.feedForwardRollerCharacterization(unit);
  }

  public Command kickerFeedForwardCharacterization(TuningUnit unit) {
    return this.kicker.feedForwardRollerCharacterization(unit);
  }
}
