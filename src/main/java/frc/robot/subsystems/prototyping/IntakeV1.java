package frc.robot.subsystems.prototyping;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.LoggedTunableNumber;
import org.robotzgarage.frc2026.subsystems.flywheel.Flywheel;
import org.robotzgarage.frc2026.subsystems.pivot.Pivot;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;

public class IntakeV1 extends SubsystemBase {
  private static final String name = "/IntakeV1";

  private static final LoggedTunableNumber intakePercent =
      new LoggedTunableNumber(name + "/IntakePercent", 0.75);
  private static final LoggedTunableNumber intakeAmps =
      new LoggedTunableNumber(name + "/IntakeAmps", 20.0);
  private static final LoggedTunableNumber intakeVoltage =
      new LoggedTunableNumber(name + "/IntakeVoltage", 6.0);

  private static final LoggedTunableNumber endeffectorGoal =
      new LoggedTunableNumber(name + "/EndeffectorGoal", 45.0);

  // Subsystems
  private final Flywheel intake;
  private final Pivot endeffector;

  // Constructor to decide if it's real or sim
  public IntakeV1(SingleRollerIO intakeIO, SingleRollerIO endeffectorIO) {
    intake = new Flywheel(name + "/Intake", intakeIO, IntakeV1Constants.flywheelConfig);
    endeffector = new Pivot(name + "/endeffector", endeffectorIO, IntakeV1Constants.pivotConfig);
  }

  public void periodic() {
    // Periodic modes
    intake.periodic();
    endeffector.periodic();
  }

  public Command runIntakePercentOut() {
    return run(
        () -> {
          double clampedPercent = MathUtil.clamp(intakePercent.get(), -1.0, 1.0);
          intake.runPercentOutput(clampedPercent);
        });
  }

  public Command runIntakeTorqueCurrent() {
    return run(
        () -> {
          double clampedAmps = MathUtil.clamp(intakeAmps.get(), 0.0, 40.0);
          intake.setTorque(Amps.of(clampedAmps));
        });
  }

  public Command runIntakeVoltageOut() {
    return run(
        () -> {
          double clampedVoltage = MathUtil.clamp(intakeVoltage.get(), -12.0, 12.0);
          intake.runVolts(clampedVoltage);
        });
  }

  public Command endeffectorGoal() {
    return run(() -> endeffector.setGoalPosition(Degrees.of(endeffectorGoal.get())));
  }

  public Command endeffectorHome() {
    return run(() -> endeffector.setGoalPosition(Degrees.of(0.0)));
  }

  public Command stop() {
    return run(
        () -> {
          intake.stop();
        });
  }
}
