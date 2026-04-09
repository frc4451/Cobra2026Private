package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.subsystems.flywheel.Flywheel;
import org.robotzgarage.frc2026.subsystems.pivot.Pivot;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;
import org.robotzgarage.frc2026.util.TuningUtils.TuningUnit;

public class Intake extends SubsystemBase {
  private static final String name = "/Intake";

  private static final LoggedTunableNumber intakePercent =
      new LoggedTunableNumber(name + "/RollersPercent", 0.75);
  private static final LoggedTunableNumber intakeAmps =
      new LoggedTunableNumber(name + "/RollersAmps", 50.0);
  private static final LoggedTunableNumber intakeVoltage =
      new LoggedTunableNumber(name + "/RollersVoltage", 9.0);

  private static final LoggedTunableNumber endeffectorStart =
      new LoggedTunableNumber(name + "/EndeffectorStart", 90.0);
  private static final LoggedTunableNumber endeffectorDeploy =
      new LoggedTunableNumber(name + "/EndeffectorDeploy", 0.0);
  private static final LoggedTunableNumber endeffectorLift =
      new LoggedTunableNumber(name + "/EndeffectorLift", 20.0);
  private static final LoggedTunableNumber endeffectorSqueeze =
      new LoggedTunableNumber(name + "/EndeffectorSqueeze", 80.0);

  // Subsystems
  private final Flywheel rollers;
  private final Pivot endeffector;

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
    endeffector = new Pivot(name + "/Endeffector", endeffectorIO, IntakeConstants.pivotConfig);
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

    Logger.recordOutput(
        name + "/Endeffector/IsNearGoal",
        endeffector.isNear(endeffector.getGoalPosition(), Degrees.of(8.0)));

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
          rollers.setTorque(Amps.of(intakeAmps.get()));
        });
  }

  // public Command runRollersVoltageOut() {
  //   return run(
  //       () -> {
  //         double clampedVoltage = MathUtil.clamp(intakeVoltage.get(), -12.0, 12.0);
  //         rollers.runVolts(clampedVoltage);
  //       });
  // }

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
        .andThen(runRollersTorqueCurrent());
  }

  /**
   * Continuously flick the Pivoting mechanism repeatedly between {@code angleA} and {@code angleB}
   * with a tolerance of {@code tolerance}. It will try to go to whichever angle is closest to the
   * current position at the start of the command. It will automatically close within the {@code
   * timeout} and end the command by setting the end state angle to {@code finalAngle}
   *
   * @param angleA lower boundary for intake flicking
   * @param angleB upper boundary for intake flicking
   * @param finalAngle goal angle for end of command
   * @param tolerance tolerance between angleA and angleB before switching goal state
   * @param timeout duration for how long to continuously flick the the intake
   */
  public Command flickIntakeRepeatedly(
      Angle angleA, Angle angleB, Angle finalAngle, Angle tolerance, Time timeout) {
    // Check for how close we are to the goal, to know where the next goal should be
    Angle goalComparisonThreshold = Degrees.of(1.0);

    return Commands.deadline(
        Commands.sequence(
            // Go to the furthest angle first so you can capitalize on
            // acceleration gains from Motion Magic
            Commands.runOnce(
                () -> {
                  double distA = Math.abs(endeffector.getPosition().minus(angleA).in(Degrees));
                  double distB = Math.abs(endeffector.getPosition().minus(angleB).in(Degrees));
                  endeffector.setGoal(distA >= distB ? angleA : angleB);
                }),
            // Force the command to stop within timeout
            // Run the flick continuously until the deadline closes the group
            Commands.deadline(
                Commands.waitTime(timeout),
                Commands.sequence(
                        // Wait until the endeffector is close to the goal position
                        Commands.waitUntil(
                            () -> endeffector.isNear(endeffector.getGoalPosition(), tolerance)),
                        // Check if we're at angleA, if we are, set the next state to angleB
                        Commands.runOnce(
                            () -> {
                              boolean atAngleA =
                                  Math.abs(endeffector.getGoalPosition().minus(angleA).in(Degrees))
                                      < goalComparisonThreshold.in(Degrees);
                              endeffector.setGoal(atAngleA ? angleB : angleA);
                            }))
                    .repeatedly()),
            // Finally, set the goal to the final angle
            Commands.runOnce(() -> endeffector.setGoal(finalAngle))),
        runRollersTorqueCurrent());
  }

  /**
   * Flick the intake between {@code angleA} and {@code angleB} X number of times with a tolerance
   * of {@code tolerance}. It will try to go to whichever angle is closest to the current position
   * at the start of the command. It will automatically close within the {@code timeout} and end the
   * command by setting the end state angle to {@code finalAngle}
   *
   * @param angleA lower boundary for intake flicking
   * @param angleB upper boundary for intake flicking
   * @param finalAngle goal angle for end of command
   * @param tolerance tolerance between angleA and angleB before switching goal state
   * @param numFlicks number of iterations it flicks
   */
  public Command flickIntakeNumberOfTimes(
      Angle angleA, Angle angleB, Angle finalAngle, Angle tolerance, int numFlicks) {
    final Angle goalComparisonThreshold = Degrees.of(1.0);

    Command[] flicks = new Command[numFlicks];
    for (int i = 0; i < numFlicks; i++) {
      flicks[i] =
          Commands.sequence(
              Commands.waitUntil(
                  () -> endeffector.isNear(endeffector.getGoalPosition(), tolerance)),
              Commands.runOnce(
                  () -> {
                    boolean atAngleA =
                        Math.abs(endeffector.getGoalPosition().minus(angleA).in(Degrees))
                            < goalComparisonThreshold.in(Degrees);
                    endeffector.setGoal(atAngleA ? angleB : angleA);
                  }));
    }

    return Commands.deadline(
        Commands.sequence(
            Commands.runOnce(
                () -> {
                  double distA = Math.abs(endeffector.getPosition().minus(angleA).in(Degrees));
                  double distB = Math.abs(endeffector.getPosition().minus(angleB).in(Degrees));
                  endeffector.setGoal(distA <= distB ? angleA : angleB);
                }),
            Commands.sequence(flicks),
            Commands.runOnce(() -> endeffector.setGoal(finalAngle))),
        runRollersTorqueCurrent());
  }

  public Command wiggelsqueeze() {
    return Commands.deadline(Commands.waitSeconds(0.75), endeffectorSqueeze())
        .andThen(Commands.sequence(stop(), endeffectorDeploy()));
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
