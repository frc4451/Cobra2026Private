package frc.robot.subsystems.prototyping;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.LoggedTunableNumber;
import org.robotzgarage.frc2026.subsystems.flywheel.Flywheel;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;

public class ClimberV1 extends SubsystemBase {
  private static final String name = "/ClimberV1";

  private static final LoggedTunableNumber climberPercent =
      new LoggedTunableNumber(name + "/ClimberPercent", 0.0);
  private static final LoggedTunableNumber climberAmps =
      new LoggedTunableNumber(name + "/ClimberAmps", 0.0);
  private static final LoggedTunableNumber climberVoltage =
      new LoggedTunableNumber(name + "/ClimberVoltage", 0.0);
  private static final LoggedTunableNumber climberGoal =
      new LoggedTunableNumber(name + "/ClimberGoal", 0.0);

  // Subsystems
  private final Flywheel climber;

  // Constructor to decide if it's real or sim
  public ClimberV1(SingleRollerIO climberIO) {
    climber = new Flywheel(name + "/Climber", climberIO, ClimberV1Constants.flywheelConfig);
  }

  public void periodic() {
    // Periodic modes
    climber.periodic();
  }

  public Command runClimberPercentOut() {
    return run(
        () -> {
          double clampedPercent = MathUtil.clamp(climberPercent.get(), -1.0, 1.0);
          climber.runPercentOutput(clampedPercent);
        });
  }

  public Command runClimberTorqueCurrent() {
    return run(
        () -> {
          double clampedAmps = MathUtil.clamp(climberAmps.get(), 0.0, 40.0);
          climber.setTorque(Amps.of(clampedAmps));
        });
  }

  public Command runClimberVoltageOut() {
    return run(
        () -> {
          double clampedVoltage = MathUtil.clamp(climberVoltage.get(), -12.0, 12.0);
          climber.runVolts(clampedVoltage);
        });
  }

  public Command setClimberGoalAngle() {
    return run(() -> climber.setGoal(Degree.of(climberGoal.get())));
  }

  public Command stop() {
    return run(
        () -> {
          climber.stop();
        });
  }
}
