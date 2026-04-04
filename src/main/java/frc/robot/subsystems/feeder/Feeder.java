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

  private static final LoggedTunableNumber spindexerAmps =
      new LoggedTunableNumber(name + "/SpindexerAmps", 20.0);
  private static final LoggedTunableNumber spindexerVoltage =
      new LoggedTunableNumber(name + "/SpindexerVoltage", 6.0);
  private static final LoggedTunableNumber spindexerPercent =
      new LoggedTunableNumber(name + "/SpindexerPercent", 0.5);

  // Subsystems
  private final Flywheel tower;
  private final Flywheel spindexer;

  // Constructor to decide if it's real or sim
  public Feeder(SingleRollerIO towerIO, SingleRollerIO spindexerIO) {
    tower = new Flywheel(name + "/Tower", towerIO, FeederConstants.towerFlywheelConfig);
    spindexer =
        new Flywheel(name + "/Spindexer", spindexerIO, FeederConstants.spindexerFlywheelConfig);
  }

  public void periodic() {
    // Periodic modes
    tower.periodic();
    spindexer.periodic();
  }

  public Command runFeederVoltage() {
    return run(
        () -> {
          tower.runVolts(towerVoltage.get());
          spindexer.runVolts(spindexerVoltage.get());
        });
  }

  public Command runSpindexerReverse() {
    return run(
        () -> {
          spindexer.runVolts(-spindexerVoltage.get());
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

  public Command runSpindexerTorqueCurrent() {
    return run(
        () -> {
          double clampedCurrent = MathUtil.clamp(spindexerAmps.get(), -100.0, 100.0);
          spindexer.runTorqueCurrent(Amps.of(clampedCurrent));
        });
  }

  public Command runSpindexerVoltage() {
    return run(
        () -> {
          double clampedVoltage = MathUtil.clamp(spindexerVoltage.get(), -12.0, 12.0);
          spindexer.runVolts(clampedVoltage);
        });
  }

  public Command runSpindexerPercentOut() {
    return run(
        () -> {
          double clampedPercent = MathUtil.clamp(spindexerPercent.get(), -1.0, 1.0);
          spindexer.runPercentOutput(clampedPercent);
        });
  }

  public Command stop() {
    return runOnce(
        () -> {
          tower.stop();
          spindexer.stop();
        });
  }

  public Command stopSpindexer() {
    return runOnce(
        () -> {
          spindexer.stop();
        });
  }

  public Command towerFeedForwardCharacterization(TuningUnit unit) {
    return this.tower.quasistaticCharacterization(unit);
  }

  public Command kickerFeedForwardCharacterization(TuningUnit unit) {
    return this.spindexer.quasistaticCharacterization(unit);
  }
}
