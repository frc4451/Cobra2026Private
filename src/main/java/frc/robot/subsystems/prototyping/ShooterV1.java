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

  private static final LoggedTunableNumber feederAmps =
      new LoggedTunableNumber("ShooterJan28/FeederAmps", 0.0);
  private static final LoggedTunableNumber feederVoltage =
      new LoggedTunableNumber("ShooterJan28/FeederVoltage", 10);
  private static final LoggedTunableNumber feederPercent =
      new LoggedTunableNumber("ShooterJan28/FeederPercent", 0.5);

  private static final LoggedTunableNumber kickerAmps =
      new LoggedTunableNumber("ShooterJan28/KickerAmps", 0.0);
  private static final LoggedTunableNumber kickerVoltage =
      new LoggedTunableNumber("ShooterJan28/KickerVoltage", 0.0);
  private static final LoggedTunableNumber kickerPercent =
      new LoggedTunableNumber("ShooterJan28/KickerPercent", 0.0);

  // Subsystems
  private final FollowRollers shooterFlyWheels;
  private final Flywheel feeder;
  private final Flywheel kicker;

  // Constructor to decide if it's real or sim
  public ShooterV1(
      FollowRollersIO shooterFlyWheelIO, SingleRollerIO feederIO, SingleRollerIO kickerIO) {
    this.shooterFlyWheels = new FollowRollers(name + "ShooterFlywheel", shooterFlyWheelIO);
    feeder = new Flywheel(name + "Feeder/", feederIO, ShooterV1Constants.feederFlywheelConfig);
    kicker = new Flywheel(name + "Kicker/", kickerIO, ShooterV1Constants.kickerFlywheelConfig);
  }

  public void periodic() {
    // Periodic modes
    feeder.periodic();
    kicker.periodic();
    shooterFlyWheels.periodic();

    if (shooterkp.hasChanged(hashCode())) {
      this.shooterFlyWheels.updatePID(shooterkp.get(), 0, 0);
    }
  }

  //   // Component voids and booleans
  //   public void runPercentFeeder(double percent) {
  //     feeder.runPercentOutput(percent);
  //   }

  //   public void runPercentSpiner(double percent) {
  //     spiner.runPercentOutput(percent);
  //   }

  public Command runShooterTuning() {
    return run(
        () -> {
          feeder.runVolts(feederVoltage.get());
          shooterFlyWheels.runVelocityTorqueCurrent(RotationsPerSecond.of(shooterVelocity.get()));
        });
  }

  public Command runShooterFlywheelPercentOut() {
    return run(
        () -> {
          double clampedPercent = MathUtil.clamp(shooterPercent.get(), -1.0, 1.0);
          shooterFlyWheels.runPercentOutput(clampedPercent);
        });
  }

  public Command runShooterFlywheelTorqueCurrent() {
    return run(
        () -> {
          // double clampedAmps = MathUtil.clamp(shooterAmps.get(), 0.0, 40.0);
          shooterFlyWheels.runTorqueCurrent(Amps.of(shooterAmps.get()));
        });
  }

  public Command runShooterFlywheelVelocityTorqueCurrent() {
    return run(
        () -> {
          double clampedVelocity = MathUtil.clamp(shooterVelocity.get(), 0.0, 200.0);
          shooterFlyWheels.runVelocityTorqueCurrent(RotationsPerSecond.of(clampedVelocity));
        });
  }

  public Command runShooterFlywheelVoltage() {
    return run(
        () -> {
          double clampedVoltage = MathUtil.clamp(shooterVoltage.get(), -12.0, 12.0);
          shooterFlyWheels.runVolts(clampedVoltage);
        });
  }

  public Command runFeederVolts() {
    return run(
        () -> {
          feeder.runVolts(feederVoltage.get());
          kicker.runVolts(kickerVoltage.get());
        });
  }

  public Command runFeederPercentOutput() {
    return run(
        () -> {
          feeder.runPercentOutput(feederPercent.get());
          kicker.runPercentOutput(kickerPercent.get());
        });
  }

  public Command runFeederTorque() {
    return run(
        () -> {
          feeder.setTorque(Amps.of(feederAmps.get()));
          kicker.setTorque(Amps.of(kickerAmps.get()));
        });
  }

  public Command runAllVoltage() {
    return run(
        () -> {
          shooterFlyWheels.runVolts(shooterVoltage.get());
          feeder.runVolts(feederVoltage.get());
          kicker.runVolts(kickerVoltage.get());
        });
  }

  public Command runAllPercent() {
    return run(
        () -> {
          shooterFlyWheels.runPercentOutput(shooterPercent.get());
          feeder.runPercentOutput(feederPercent.get());
          kicker.runPercentOutput(kickerPercent.get());
        });
  }

  public Command runAllTorqueCurrent() {
    return run(
        () -> {
          shooterFlyWheels.runTorqueCurrent(Amps.of(shooterAmps.get()));
          feeder.setTorque(Amps.of(shooterAmps.get()));
          kicker.setTorque(Amps.of(kickerAmps.get()));
        });
  }

  public Command stop() {
    return run(
        () -> {
          shooterFlyWheels.stop();
          feeder.stop();
          kicker.stop();
        });
  }
}
