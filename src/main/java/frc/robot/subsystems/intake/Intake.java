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
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRoller;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;
import org.robotzgarage.frc2026.util.TuningUtils.TuningUnit;

public class Intake extends SubsystemBase {
  private static final String name = "/Intake";

  private static final LoggedTunableNumber intakePercent =
      new LoggedTunableNumber(name + "/RollersPercent", 0.75);
  private static final LoggedTunableNumber intakeAmps =
      new LoggedTunableNumber(name + "/RollersAmps", 20.0);
  private static final LoggedTunableNumber intakeVoltage =
      new LoggedTunableNumber(name + "/RollersVoltage", 9.0);

  private static final LoggedTunableNumber endeffectorStart =
      new LoggedTunableNumber(name + "/EndeffectorStart", 90.0);
  private static final LoggedTunableNumber endeffectorDeploy =
      new LoggedTunableNumber(name + "/EndeffectorDeploy", 0.0);
  private static final LoggedTunableNumber endeffectorDeployVolts =
      new LoggedTunableNumber(name + "/EndeffectorDeployVolts", 0.0);
  private static final LoggedTunableNumber endeffectorLift =
      new LoggedTunableNumber(name + "/EndeffectorLift", 20.0);
  private static final LoggedTunableNumber endeffectorSqueeze =
      new LoggedTunableNumber(name + "/EndeffectorSqueeze", 75.0);

  // Subsystems
  private final Flywheel rollers;
  private final SingleRoller endeffector;

  public enum EndeffectorState {
    STARTING(Degrees.of(endeffectorStart.get())),
    DEPLOYED(Degrees.of(endeffectorDeploy.get())),
    LIFTED(Degrees.of(endeffectorLift.get())),
    SQUEEZE(Degrees.of(endeffectorSqueeze.get()));

    private Angle angle;

    private EndeffectorState(Angle angle) {
      this.angle = angle;
    }
  }

  private EndeffectorState currentState = EndeffectorState.STARTING;

  // Constructor to decide if it's real or sim
  public Intake(SingleRollerIO intakeIO, SingleRollerIO endeffectorIO) {
    rollers = new Flywheel(name + "/Intake", intakeIO, IntakeConstants.flywheelConfig);
    endeffector = new Pivot(name + "/endeffector", endeffectorIO, IntakeConstants.pivotConfig);
  }

  public void periodic() {
    // switch (currentState) {
    //   case STARTING:
    //     endeffectorStart();
    //     break;
    //   case DEPLOYED:
    //     endeffectorDeploy();
    //     break;
    //   case LIFTED:
    //     endeffectorLift();
    //     break;
    //   case SQUEEZE:
    //     endeffectorSqueeze();
    //     break;
    //   default:
    //     break;
    // }
    // Periodic modes
    rollers.periodic();
    endeffector.periodic();

    // endeffector.setGoal(endeffector.getGoalPosition());
  }

  public Command runRollersPercentOut() {
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

  public Command endeffectorStart() {
    return runOnce(() -> endeffector.setGoal(Degrees.of(endeffectorStart.get())));
  }

  public Command endeffectorDeploy() {
    return runOnce(() -> endeffector.setGoal(Degrees.of(endeffectorDeploy.get())));
  }

  public Command endeffectorLift() {
    return runOnce(() -> endeffector.setGoal(Degrees.of(endeffectorLift.get())));
  }

  public Command endeffectorSqueeze() {
    return runOnce(() -> endeffector.setGoal(Degrees.of(endeffectorSqueeze.get())))
        .andThen(runRollersVoltageOut());
  }

  // public Command endeffectorpush() {
  //   return runEnd(() -> endeffector.runVolts(-0.5), () -> endeffector.stop());
  // }

  public Trigger isNearGoal(Angle tolerance) {
    return new Trigger(
        () -> this.endeffector.isNear(this.endeffector.getGoalPosition(), tolerance));
  }

  public void setCurrentState(EndeffectorState state) {
    if (currentState != state) {
      currentState = state;
    }
  }

  public Command stop() {
    return runOnce(() -> rollers.stop());
  }

  public Command stopPivot() {
    return runOnce(() -> endeffector.stop());
  }

  public Command rollersFeedForwardCharacterization(TuningUnit unit) {
    return this.rollers.quasistaticCharacterization(unit);
  }

  public Command endeffectorFeedForwardCharacterization(TuningUnit unit) {
    return this.endeffector.quasistaticCharacterization(unit);
  }
}
