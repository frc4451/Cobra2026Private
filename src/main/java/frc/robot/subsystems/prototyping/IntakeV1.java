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

  private static final LoggedTunableNumber IntakePercent =
      new LoggedTunableNumber(name + "/IntakePercent", 0.0);
  private static final LoggedTunableNumber IntakeAmps =
      new LoggedTunableNumber(name + "/IntakeAmps", 0.0);
  private static final LoggedTunableNumber IntakeVoltage =
      new LoggedTunableNumber(name + "/IntakeVoltage", 0.0);

  private static final LoggedTunableNumber EndeffectorGoal =
      new LoggedTunableNumber(name + "/EndeffectorGoal", 0.0);

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
          double clampedPercent = MathUtil.clamp(IntakePercent.get(), -1.0, 1.0);
          intake.runPercentOutput(clampedPercent);
        });
  }

  public Command runIntakeTorqueCurrent() {
    return run(
        () -> {
          double clampedAmps = MathUtil.clamp(IntakeAmps.get(), 0.0, 40.0);
          intake.setTorque(Amps.of(clampedAmps));
        });
  }

  public Command runIntakeVoltageOut() {
    return run(
        () -> {
          double clampedVoltage = MathUtil.clamp(IntakeVoltage.get(), -12.0, 12.0);
          intake.runVolts(clampedVoltage);
        });
  }

  public Command setEndeffectorGoal() {
    return run(() -> endeffector.setGoalPosition(Degrees.of(EndeffectorGoal.get())));
  }

  public Command stop() {
    return run(
        () -> {
          intake.stop();
        });
  }
}
