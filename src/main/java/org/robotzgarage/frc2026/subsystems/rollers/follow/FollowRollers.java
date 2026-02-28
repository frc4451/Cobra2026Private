package org.robotzgarage.frc2026.subsystems.rollers.follow;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Getter;
import org.littletonrobotics.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.util.MotorConfigs.GenericFeedForwardConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.GenericFeedbackConfig;

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
}
