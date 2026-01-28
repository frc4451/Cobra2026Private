package frc.robot.subsystems.prototype;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.LoggedTunableNumber;
import org.robotzgarage.frc2026.subsystems.flywheel.Flywheel;
import org.robotzgarage.frc2026.subsystems.rollers.follow.FollowRollersIO;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;

public class ShooterJan28 extends SubsystemBase {
  private final String name = "ShooterJan28/";

  private static final LoggedTunableNumber shooterAmps =
      new LoggedTunableNumber("ShooterJan28/ShooterAmps");
  private static final LoggedTunableNumber shooterVoltage =
      new LoggedTunableNumber("ShooterJan28/ShooterVoltage");
  private static final LoggedTunableNumber shooterPercent =
      new LoggedTunableNumber("ShooterJan28/ShooterPercent");

  private static final LoggedTunableNumber feederAmps =
      new LoggedTunableNumber("ShooterJan28/FeederAmps");
  private static final LoggedTunableNumber feederVoltage =
      new LoggedTunableNumber("ShooterJan28/FeederVoltage");
  private static final LoggedTunableNumber feederPercent =
      new LoggedTunableNumber("ShooterJan28/FeederPercent");

  // Subsystems
  private final FollowRollersIO shooterFlyWheelIO;
  private final Flywheel feeder;

  // Constructor to decide if it's real or sim
  public ShooterJan28(FollowRollersIO shooterFlyWheelIO, SingleRollerIO feederIO) {
    this.shooterFlyWheelIO = shooterFlyWheelIO;
    feeder = new Flywheel(name + "feeder/", feederIO, ShooterJan28Constants.feederFlywheelConfig);
  }

  public void periodic() {
    // Periodic modes
    feeder.periodic();
  }

  //   // Component voids and booleans
  //   public void runPercentFeeder(double percent) {
  //     feeder.runPercentOutput(percent);
  //   }

  //   public void runPercentSpiner(double percent) {
  //     spiner.runPercentOutput(percent);
  //   }

  public Command runShooterFlywheelPercentOut() {
    return run(
        () -> {
          double clampedPercent = MathUtil.clamp(shooterPercent.get(), -1.0, 1.0);
          shooterFlyWheelIO.runPercentOutput(clampedPercent);
        });
  }

  public Command runShooterFlywheelTorqueCurrent() {
    return run(
        () -> {
          double clampedAmps = MathUtil.clamp(shooterAmps.get(), 0.0, 40.0);
          shooterFlyWheelIO.runTorqueCurrent(Amps.of(clampedAmps));
        });
  }

  public Command runShooterFlywheelVoltage() {
    return run(
        () -> {
          double clampedVoltage = MathUtil.clamp(shooterVoltage.get(), -12.0, 12.0);
          shooterFlyWheelIO.runVolts(clampedVoltage);
        });
  }

  public Command runFeederVolts() {
    return run(() -> feeder.runVolts(feederVoltage.get()));
  }

  public Command runFeederPercentOutput() {
    return run(() -> feeder.runPercentOutput(feederPercent.get()));
  }

  public Command runFeederTorque() {
    return run(() -> feeder.setTorque(Amps.of(feederAmps.get())));
  }

  public Command runAllVolts() {
    return run(
        () -> {
          shooterFlyWheelIO.runVolts(shooterVoltage.get());
          feeder.runVolts(feederVoltage.get());
        });
  }

  public Command runAllPercent() {
    return run(
        () -> {
          shooterFlyWheelIO.runPercentOutput(shooterPercent.get());
          feeder.runPercentOutput(feederPercent.get());
        });
  }

  public Command runAllTorque() {
    return run(
        () -> {
          shooterFlyWheelIO.runTorqueCurrent(Amps.of(shooterAmps.get()));
          feeder.setTorque(Amps.of(shooterAmps.get()));
        });
  }

  public Command stop() {
    return run(
        () -> {
          shooterFlyWheelIO.stop();
          feeder.stop();
        });
  }
}
