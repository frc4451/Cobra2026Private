package org.robotzgarage.frc2026.subsystems.rollers.single;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.util.MotorConfigs.GenericFeedForwardConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.GenericFeedbackConfig;

public class SingleRoller {
  protected final String name;
  protected final SingleRollerIO io;

  private final LoggedTunableNumber logTunablekp;
  private final LoggedTunableNumber logTunableki;
  private final LoggedTunableNumber logTunablekd;
  private final LoggedTunableNumber logTunableks;
  private final LoggedTunableNumber logTunablekg;
  private final LoggedTunableNumber logTunablekv;
  private final LoggedTunableNumber logTunableka;

  protected final SingleRollerIOInputsAutoLogged inputs = new SingleRollerIOInputsAutoLogged();

  private final Alert disconnected;

  public SingleRoller(String name, SingleRollerIO io) {
    this.name = name;
    this.io = io;

    logTunablekp = new LoggedTunableNumber(name + "/FeedBack/kP", io.readFeedBack().p());
    logTunableki = new LoggedTunableNumber(name + "/FeedBack/kI", io.readFeedBack().i());
    logTunablekd = new LoggedTunableNumber(name + "/FeedBack/kD", io.readFeedBack().d());
    logTunableks = new LoggedTunableNumber(name + "/FeedForward/kS", io.readFeedForward().s());
    logTunablekg = new LoggedTunableNumber(name + "/FeedForward/kG", io.readFeedForward().g());
    logTunablekv = new LoggedTunableNumber(name + "/FeedForward/kV", io.readFeedForward().v());
    logTunableka = new LoggedTunableNumber(name + "/FeedForward/kA", io.readFeedForward().a());

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnected.set(!inputs.connected);

    if (DriverStation.isDisabled()) {
      io.stop();
    }

    if (
      logTunablekp.hasChanged(hashCode())
        || logTunableki.hasChanged(hashCode())
        || logTunablekd.hasChanged(hashCode())) {
      this.io.updateFeedBack(GenericFeedbackConfig.builder()
        .p(logTunablekp.get())
        .i(logTunableki.get())
        .d(logTunablekd.get())
        .build()
        );
    }

    if (logTunableks.hasChanged(hashCode()) 
    || logTunablekg.hasChanged(hashCode()) 
    || logTunablekv.hasChanged(hashCode())
    || logTunableka.hasChanged(hashCode())) {
      this.io.updateFeedForward(GenericFeedForwardConfig.builder()
        .s(logTunableks.get())
        .g(logTunablekg.get())
        .v(logTunablekv.get())
        .a(logTunableka.get())
        .build()
        );
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

  public void stop() {
    io.stop();
  }

  public void setGoal(Angle position) {
    io.setGoal(position);
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
