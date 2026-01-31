package org.robotzgarage.frc2026.subsystems.rollers.follow;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

public class FollowRollers {
  protected final String name;
  protected final FollowRollersIO io;

  protected final FollowRollersMagicIOInputsAutoLogged inputs =
      new FollowRollersMagicIOInputsAutoLogged();

  private final Alert leaderDisconnected;
  private final Alert followerDisconnected;

  public FollowRollers(String name, FollowRollersIO io) {
    this.name = name;
    this.io = io;

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
  }

  public void runVolts(double inputsVolts) {
    io.runVolts(inputsVolts);
  }

  public void runPercentOutput(double percent) {
    io.runPercentOutput(percent);
  }

  public void runTorqueCurrent(Current amps) {
    io.runTorqueCurrent(amps);
  }

  public void setGoal(Angle position) {
    io.setGoal(position);
  }

  public void stop() {
    io.stop();
  }
}
