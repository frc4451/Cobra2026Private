package org.robotzgarage.frc2026.subsystems.rollers.single;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import org.robotzgarage.frc2026.util.MotorConfigs.CANCoderConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfigSim;

public class SingleRollerIOSimTalonFXWithCANCoder extends SingleRollerIOTalonFXWithCANCoder
    implements AutoCloseable {
  private final DCMotorSim motorSim;

  /** Keep track of CTRE Sim updates in separate thread */
  private final Notifier simNotifier;

  private double lastUpdateTimestamp;

  private double lastTimestampDelta;

  public SingleRollerIOSimTalonFXWithCANCoder(
      TalonFXConfigSim config, CANCoderConfig cancoderConfig) {
    super(config.config(), cancoderConfig);

    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(config.dcMotor(), config.moi(), totalReduction),
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
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    motorSim.setInputVoltage(talon.getSimState().getMotorVoltage());

    // Update simulation BEFORE reading state
    motorSim.update(delta);

    // Now read updated values and write to TalonFX sim state
    // Convert mechanism rotations to rotor rotations (multiply by reduction)
    talon.getSimState().setRawRotorPosition(motorSim.getAngularPosition().times(totalReduction));
    talon.getSimState().setRotorVelocity(motorSim.getAngularVelocity().times(totalReduction));

    // Update battery simulation with current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(motorSim.getCurrentDrawAmps()));
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
