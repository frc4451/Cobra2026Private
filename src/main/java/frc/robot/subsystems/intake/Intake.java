package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.LoggedTunableNumber;
import org.robotzgarage.frc2026.subsystems.flywheel.Flywheel;
import org.robotzgarage.frc2026.subsystems.pivot.Pivot;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;

public class Intake extends SubsystemBase {
  private static final String name = "/Intake";

  private static final LoggedTunableNumber intakePercent =
      new LoggedTunableNumber(name + "/RollersPercent", 0.75);
  private static final LoggedTunableNumber intakeAmps =
      new LoggedTunableNumber(name + "/RollersAmps", 20.0);
  private static final LoggedTunableNumber intakeVoltage =
      new LoggedTunableNumber(name + "/RollersVoltage", 6.0);

  private static final LoggedTunableNumber endeffectorGoal =
      new LoggedTunableNumber(name + "/EndeffectorGoal", 45.0);

  // Subsystems
  private final Flywheel rollers;
  private final Pivot endeffector;

  // Constructor to decide if it's real or sim
  public Intake(SingleRollerIO intakeIO, SingleRollerIO endeffectorIO) {
    rollers = new Flywheel(name + "/Intake", intakeIO, IntakeConstants.flywheelConfig);
    endeffector = new Pivot(name + "/endeffector", endeffectorIO, IntakeConstants.pivotConfig);
  }

  public void periodic() {
    // Periodic modes
    rollers.periodic();
    endeffector.periodic();
  }

  public Command runRollersPercentOut() {
    return run(
        () -> {
          double clampedPercent = MathUtil.clamp(intakePercent.get(), -1.0, 1.0);
          rollers.runPercentOutput(clampedPercent);
        });
  }

  public Command runRollersPercentOut(double percentOut) {
    return run(
        () -> {
          double clampedPercent = MathUtil.clamp(intakePercent.get(), -1.0, 1.0);
          rollers.runPercentOutput(clampedPercent);
        });
  }

  public Command runRollersTorqueCurrent() {
    return run(
        () -> {
          double clampedAmps = MathUtil.clamp(intakeAmps.get(), 0.0, 40.0);
          rollers.setTorque(Amps.of(clampedAmps));
        });
  }

  public Command runRollersVoltageOut() {
    return run(
        () -> {
          double clampedVoltage = MathUtil.clamp(intakeVoltage.get(), -12.0, 12.0);
          rollers.runVolts(clampedVoltage);
        });
  }

  public Command endeffectorGoal() {
    return run(() -> endeffector.setGoalPosition(Degrees.of(endeffectorGoal.get())));
  }

  public Command endeffectorGoal(Angle angle) {
    return run(() -> endeffector.setGoalPosition(angle));
  }

  public Command retractEndeffector() {
    return run(() -> endeffectorGoal(Degrees.of(90.0)));
  }

  public Trigger isNearGoal(Angle tolerance) {
    return new Trigger(() -> this.endeffector.isNear(endeffector.getGoalPosition(), tolerance));
  }

  public Command endeffectorHome() {
    return run(() -> endeffector.setGoalPosition(Degrees.of(0.0)));
  }

  public Command stop() {
    return runOnce(
        () -> {
          rollers.stop();
        });
  }
}
