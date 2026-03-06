package org.robotzgarage.frc2026.subsystems.rollers.follow;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import lombok.Getter;
import org.littletonrobotics.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.util.MotorConfigs.GenericFeedForwardConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.GenericFeedbackConfig;
import org.robotzgarage.frc2026.util.TuningUtils.TuningUnit;

public class FollowRollers {
  protected final String name;
  protected final FollowRollersIO io;

  private final LoggedTunableNumber logTunablekp;
  private final LoggedTunableNumber logTunableki;
  private final LoggedTunableNumber logTunablekd;
  private final LoggedTunableNumber logTunableks;
  private final LoggedTunableNumber logTunablekg;
  private final LoggedTunableNumber logTunablekv;
  private final LoggedTunableNumber logTunableka;

  @Getter
  protected final FollowRollersMagicIOInputsAutoLogged inputs =
      new FollowRollersMagicIOInputsAutoLogged();

  private final Alert leaderDisconnected;
  private final Alert followerDisconnected;

  public FollowRollers(String name, FollowRollersIO io) {
    this.name = name;
    this.io = io;

    logTunablekp = new LoggedTunableNumber(name + "/FeedBack/kP", io.readFeedBack().p());
    logTunableki = new LoggedTunableNumber(name + "/FeedBack/kI", io.readFeedBack().i());
    logTunablekd = new LoggedTunableNumber(name + "/FeedBack/kD", io.readFeedBack().d());
    logTunableks = new LoggedTunableNumber(name + "/FeedForward/kS", io.readFeedForward().s());
    logTunablekg = new LoggedTunableNumber(name + "/FeedForward/kG", io.readFeedForward().g());
    logTunablekv = new LoggedTunableNumber(name + "/FeedForward/kV", io.readFeedForward().v());
    logTunableka = new LoggedTunableNumber(name + "/FeedForward/kA", io.readFeedForward().a());

    leaderDisconnected = new Alert(name + " leader motor disconnected!", Alert.AlertType.kWarning);
    followerDisconnected =
        new Alert(name + " follower motor disconnected!", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    leaderDisconnected.set(!inputs.leaderConnected);
    followerDisconnected.set(!inputs.followerConnected);

    if (DriverStation.isDisabled()) {
      io.stop();
    }

    if (logTunablekp.hasChanged(hashCode())
        || logTunableki.hasChanged(hashCode())
        || logTunablekd.hasChanged(hashCode())) {
      this.io.updateFeedBack(
          GenericFeedbackConfig.builder()
              .p(logTunablekp.get())
              .i(logTunableki.get())
              .d(logTunablekd.get())
              .build());
    }

    if (logTunableks.hasChanged(hashCode())
        || logTunablekg.hasChanged(hashCode())
        || logTunablekv.hasChanged(hashCode())
        || logTunableka.hasChanged(hashCode())) {
      this.io.updateFeedForward(
          GenericFeedForwardConfig.builder()
              .s(logTunableks.get())
              .g(logTunablekg.get())
              .v(logTunablekv.get())
              .a(logTunableka.get())
              .build());
    }
  }

  public void runVolts(double inputsVolts) {
    io.runVolts(inputsVolts);
  }

  public void runPercentOutput(double percent) {
    io.runPercentOutput(percent);
  }

  public void runTorqueCurrent(Current current) {
    io.runTorqueCurrent(current);
  }

  public void setVelocityTorqueCurrent(AngularVelocity velocity) {
    io.setVelocityTorqueCurrent(velocity);
  }

  public void setGoal(Angle position) {
    io.setGoal(position);
  }

  public void stop() {
    io.stop();
  }

  public GenericFeedbackConfig readFeedBack() {
    return io.readFeedBack();
  }

  public void updateFeedBack(GenericFeedbackConfig fbConfig) {
    io.updateFeedBack(fbConfig);
  }

  public GenericFeedForwardConfig readFeedForward() {
    return io.readFeedForward();
  }

  public void updateFeedForward(GenericFeedForwardConfig ffConfig) {
    io.updateFeedForward(ffConfig);
  }

  /**
   * Steady-state FF characterization that matches the Phoenix TorqueCurrentFOC tuning guide.
   *
   * <p>Instead of a continuous ramp (which contaminates kS/kV with kA*acceleration), this applies a
   * fixed input at each step and waits for velocity to fully settle before recording. The
   * regression is then fit through true steady-state (zero acceleration) points, so kS and kV are
   * not biased by inertia.
   *
   * <p>Tune {@code INPUT_STEP}, {@code NUM_STEPS}, and {@code SETTLE_TIME_SECS} for your mechanism.
   * A heavier flywheel needs a longer settle time; lower INPUT_STEP gives more data points.
   */
  public Command steadyStateCharacterization(TuningUnit unit) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> unitSamples = new LinkedList<>();

    // --- Tuning parameters ---
    // Total sweep: INPUT_STEP * NUM_STEPS amps/volts at INPUT_STEP increments
    final double INPUT_STEP = 1.0; // amps or volts per step
    final int NUM_STEPS = 40; // number of steps (1 → 20 A/V)
    final double SETTLE_TIME_SECS = 1.5; // seconds to wait per step for velocity to stabilize
    final double MIN_VELOCITY_RPS = 0.1; // skip steps where the mechanism hasn't started moving

    GenericFeedForwardConfig snapshotFFConfig = this.readFeedForward();
    GenericFeedbackConfig snapshotFBConfig = this.readFeedBack();

    // Build one "apply → wait → record" block per step
    Command[] stepCommands = new Command[NUM_STEPS];
    for (int i = 0; i < NUM_STEPS; i++) {
      final double input = (i + 1) * INPUT_STEP;
      stepCommands[i] =
          Commands.sequence(
              // Apply fixed input and let the motor controller latch it while we wait
              Commands.runOnce(
                  () -> {
                    if (unit == TuningUnit.AMPS) {
                      this.runTorqueCurrent(Amps.of(input));
                    } else {
                      this.runVolts(input);
                    }
                  }),
              Commands.waitSeconds(SETTLE_TIME_SECS),
              // Record now that velocity has had time to settle to steady state
              Commands.runOnce(
                  () -> {
                    double velocity = this.inputs.leaderVelocity.in(RotationsPerSecond);
                    if (Math.abs(velocity) > MIN_VELOCITY_RPS) {
                      velocitySamples.add(velocity);
                      unitSamples.add(input);
                      System.out.printf(
                          "Recorded: %.2f %s → %.4f rot/s%n",
                          input, unit == TuningUnit.AMPS ? "A" : "V", velocity);
                    } else {
                      System.out.printf(
                          "Skipped: %.2f %s → %.4f rot/s (below threshold, still finding kS)%n",
                          input, unit == TuningUnit.AMPS ? "A" : "V", velocity);
                    }
                  }));
    }

    return Commands.sequence(
        Commands.print("*** Steady-State FF Characterization: zeroing gains ***"),
        Commands.runOnce(
            () -> {
              this.updateFeedForward(GenericFeedForwardConfig.getEmptyConfig());
              this.updateFeedBack(GenericFeedbackConfig.getEmptyConfig());
              velocitySamples.clear();
              unitSamples.clear();
            }),
        Commands.sequence(stepCommands),
        Commands.runOnce(
            () -> {
              if (unit == TuningUnit.AMPS) {
                this.runTorqueCurrent(Amps.zero());
              } else {
                this.runVolts(0.0);
              }

              int n = velocitySamples.size();
              if (n < 2) {
                System.out.println(
                    "Not enough data! Lower INPUT_STEP, increase NUM_STEPS, or check mechanism.");
                return;
              }

              double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumX2 = 0.0;
              for (int i = 0; i < n; i++) {
                double v = velocitySamples.get(i);
                double u = unitSamples.get(i);
                sumX += v;
                sumY += u;
                sumXY += v * u;
                sumX2 += v * v;
              }

              // Linear regression through true steady-state points: unit = kS + kV * velocity
              // No kA contamination because acceleration = 0 at each recorded sample
              double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
              double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

              NumberFormat formatter = new DecimalFormat("#0.000000");
              String unitLabel = unit == TuningUnit.AMPS ? "amps" : "volts";
              String kVUnit = unit == TuningUnit.AMPS ? "amps/(rot/s)" : "volts/(rot/s)";
              System.out.println("*** Steady-State FF Results ***");
              System.out.println("Samples collected: " + n);
              System.out.println("kS (" + unitLabel + "): " + formatter.format(kS));
              System.out.println("kV (" + kVUnit + "): " + formatter.format(kV));

              Logger.recordOutput("TuningRoutine/SteadyState/" + name + "/kS", kS);
              Logger.recordOutput("TuningRoutine/SteadyState/" + name + "/kV", kV);
            }),
        Commands.print("Resetting FeedForward and Feedback to snapshots"),
        Commands.runOnce(
            () -> {
              this.updateFeedForward(snapshotFFConfig);
              this.updateFeedBack(snapshotFBConfig);
            }),
        Commands.print("Finished steady-state characterization for '" + this.name + "'"));
  }

  public Command quasistaticCharacterization(TuningUnit unit) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> unitSamples = new LinkedList<>();
    Timer timer = new Timer();

    // Minimum velocity (rot/s) before we start collecting samples.
    // Pre-motion samples (velocity=0, input ramping up) would corrupt the regression by
    // pulling kS high and flattening kV — we want only the moving steady-state data.
    final double VELOCITY_THRESHOLD_RPS = 0.1;

    GenericFeedForwardConfig snapshotFFConfig = this.readFeedForward();
    GenericFeedbackConfig snapshotFBConfig = this.readFeedBack();

    return Commands.sequence(
        Commands.print("Resetting FeedForward and Feedback Configs"),
        Commands.runOnce(
            () -> {
              this.updateFeedForward(GenericFeedForwardConfig.getEmptyConfig());
              this.updateFeedBack(GenericFeedbackConfig.getEmptyConfig());
            }),
        Commands.print("Resetting sample collection data"),
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              unitSamples.clear();
              timer.restart();
            }),
        Commands.print("Start Data Collection, observe robot readings"),
        Commands.run(
                () -> {
                  double input = timer.get() * Constants.tuningFFRampRate;

                  if (unit == TuningUnit.AMPS) {
                    this.runTorqueCurrent(Amps.of(input));
                  } else if (unit == TuningUnit.VOLTS) {
                    this.runVolts(input);
                  }

                  double velocity = this.inputs.leaderVelocity.in(RotationsPerSecond);
                  // Only record samples once the mechanism is actually moving so the linear
                  // regression fits the moving region: unit = kS + kV * velocity
                  if (Math.abs(velocity) > VELOCITY_THRESHOLD_RPS) {
                    unitSamples.add(input);
                    velocitySamples.add(velocity);
                  }
                })
            .finallyDo(
                () -> {
                  if (unit == TuningUnit.AMPS) {
                    this.runTorqueCurrent(Amps.zero());
                  } else if (unit == TuningUnit.VOLTS) {
                    this.runVolts(0.0);
                  }

                  int n = velocitySamples.size();
                  if (n < 2) {
                    System.out.println(
                        "Not enough data collected! Run longer or lower VELOCITY_THRESHOLD_RPS.");
                    return;
                  }

                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;

                  for (int i = 0; i < n; i++) {
                    double v = velocitySamples.get(i);
                    double iUnit = unitSamples.get(i);

                    sumX += v;
                    sumY += iUnit;
                    sumXY += v * iUnit;
                    sumX2 += v * v;
                  }

                  // Linear regression: unit = kS + kV * velocity
                  // kS (y-intercept): input to barely overcome static friction
                  // kV (slope):       input per rot/s of drag compensation
                  //   - AMPS mode: both kS and kV are in amps / (amps per rot/s)
                  //     These map directly to TorqueCurrentFOC Slot kS and kV in Phoenix
                  //   - VOLTS mode: kS in volts, kV in volts per rot/s
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.000000");
                  String unitLabel = unit == TuningUnit.AMPS ? "amps" : "volts";
                  String kVUnit = unit == TuningUnit.AMPS ? "amps/(rot/s)" : "volts/(rot/s)";
                  System.out.println("********** Flywheel FF Results **********");
                  System.out.println("Samples collected: " + n);
                  System.out.println("kS (" + unitLabel + "): " + formatter.format(kS));
                  System.out.println("kV (" + kVUnit + "): " + formatter.format(kV));

                  Logger.recordOutput("TuningRoutine/" + name + "/kS", kS);
                  Logger.recordOutput("TuningRoutine/" + name + "/kV", kV);
                })
            .andThen(
                Commands.sequence(
                    Commands.print("Resetting FeedForward and Feedback to snapshots"),
                    Commands.runOnce(
                        () -> {
                          this.updateFeedForward(snapshotFFConfig);
                          this.updateFeedBack(snapshotFBConfig);
                        }),
                    Commands.print("Finished Flywheel '" + this.name + "' Characterization"))));
  }
}
