package org.robotzgarage.frc2026.subsystems.rollers.follow;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfigFollowerSim;

public class FollowRollersIOSimTalonFX extends FollowRollersIOTalonFX implements AutoCloseable {
  private final DCMotorSim leaderSim;
  private final DCMotorSim followerSim;

  /** Keep track of CTRE Sim updates in separate thread */
  private final Notifier simNotifier;

  private double lastUpdateTimestamp;

  private double lastTimestampDelta;

  public FollowRollersIOSimTalonFX(TalonFXConfigFollowerSim config) {
    super(config.hardware());

    leaderSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(config.dcMotor(), config.moi(), reduction),
            config.dcMotor());

    followerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(config.dcMotor(), config.moi(), reduction),
            config.dcMotor());

    // Initialize timestamp before starting simulation updates
    lastUpdateTimestamp = Timer.getFPGATimestamp();

    simNotifier =
        new Notifier(
            () -> {
              updateSimState();
            });
    simNotifier.startPeriodic(0.005);
  }

  private void updateSimState() {
    // Calculate time delta first
    double timestamp = Timer.getFPGATimestamp();
    double delta = timestamp - lastUpdateTimestamp;
    this.lastUpdateTimestamp = timestamp;
    this.lastTimestampDelta = delta;

    // Set supply voltage and get commanded motor voltage
    leader.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    leaderSim.setInputVoltage(leader.getSimState().getMotorVoltage());

    follower.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    followerSim.setInputVoltage(follower.getSimState().getMotorVoltage());

    // Update simulation BEFORE reading state
    leaderSim.update(delta);
    followerSim.update(delta);

    // Now read updated values and write to TalonFX sim state
    // Convert mechanism rotations to rotor rotations (multiply by reduction)
    leader.getSimState().setRawRotorPosition(leaderSim.getAngularPosition().times(reduction));
    leader.getSimState().setRotorVelocity(leaderSim.getAngularVelocity().times(reduction));

    follower.getSimState().setRawRotorPosition(followerSim.getAngularPosition().times(reduction));
    follower.getSimState().setRotorVelocity(followerSim.getAngularVelocity().times(reduction));

    // Update battery simulation with current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(leaderSim.getCurrentDrawAmps()));
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(followerSim.getCurrentDrawAmps()));
  }

  @Override
  public void close() {
    simNotifier.close();
  }

  @Override
  public double getLastTimestampDelta() {
    return this.lastTimestampDelta;
  }
}
