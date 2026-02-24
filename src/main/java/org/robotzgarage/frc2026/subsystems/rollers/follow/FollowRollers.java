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

  public Command feedForwardCharacterization(TuningUnit unit) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> unitSamples = new LinkedList<>();
    Timer timer = new Timer();

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

                  unitSamples.add(input);
                  velocitySamples.add(this.inputs.leaderVelocity.in(RotationsPerSecond));
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
                    System.out.println("Not enough data collected!");
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

                  // Linear regression:
                  // current = kS + kV * velocity
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);

                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.000000");
                  String unitLabel = unit == TuningUnit.AMPS ? "amps" : "volts";
                  String tuningConstant = unit == TuningUnit.AMPS ? "kA" : "kV";
                  System.out.println("********** Flywheel FF Results **********");
                  System.out.println("kS (" + unitLabel + "): " + formatter.format(kS));
                  System.out.println(
                      tuningConstant + " (" + unitLabel + " per rot/s): " + formatter.format(kV));
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
