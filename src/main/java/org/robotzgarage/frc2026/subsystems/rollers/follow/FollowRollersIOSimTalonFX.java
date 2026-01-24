package org.robotzgarage.frc2026.subsystems.rollers.follow;

import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.LoggedRobot;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfigFollowerSim;

public class FollowRollersIOSimTalonFX extends FollowRollersIOTalonFX {
  private final DCMotorSim leaderSim;
  private final DCMotorSim followerSim;

  /** Motion Magic does _not_ work in Sim, so we use Position Voltage in the meantime */
  private final PositionVoltage positionVoltage;

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

    positionVoltage =
        new PositionVoltage(0).withSlot(0).withEnableFOC(config.hardware().leader().foc());
  }

  @Override
  public void updateInputs(FollowRollersMagicIOInputs inputs) {
    // Still do everything as expected for TalonFXS
    super.updateInputs(inputs);

    // Update motor output from sim state
    leaderSim.setInput(leader.getSimState().getMotorVoltage());
    leaderSim.update(LoggedRobot.defaultPeriodSecs);

    followerSim.setInput(follower.getSimState().getMotorVoltage());
    followerSim.update(LoggedRobot.defaultPeriodSecs);

    // (WIP) Take DCMotorSim configs and set them to TalonFXS Sim
    leader.getSimState().setRawRotorPosition(leaderSim.getAngularPositionRotations() / reduction);
    leader
        .getSimState()
        .setRotorVelocity(
            Units.radiansToRotations(leaderSim.getAngularVelocityRadPerSec() / reduction));

    follower
        .getSimState()
        .setRawRotorPosition(followerSim.getAngularPositionRotations() / reduction);
    follower
        .getSimState()
        .setRotorVelocity(
            Units.radiansToRotations(followerSim.getAngularVelocityRadPerSec() / reduction));
  }

  @Override
  public void setGoal(Angle position) {
    positionGoalRotations = position.times(reduction);
    leader.setControl(positionVoltage.withPosition(positionGoalRotations));
  }
}
