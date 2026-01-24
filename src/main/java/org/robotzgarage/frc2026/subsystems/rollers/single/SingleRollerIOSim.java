package org.robotzgarage.frc2026.subsystems.rollers.single;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.LoggedRobot;
import org.robotzgarage.frc2026.subsystems.rollers.feedforward_controller.FeedforwardController;
import org.robotzgarage.frc2026.util.MotorConfigs.SimMotorConfig;

public class SingleRollerIOSim implements SingleRollerIO {
  private final DCMotorSim sim;

  private final ProfiledPIDController controller;

  private final FeedforwardController ff;

  private boolean isClosedLoop = false;

  public SingleRollerIOSim(SimMotorConfig config) {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                config.motorModel(), config.moi(), config.reduction()),
            config.motorModel());

    this.ff = config.ff();
    this.controller =
        new ProfiledPIDController(
            config.kP(),
            config.kI(),
            config.kD(),
            new TrapezoidProfile.Constraints(config.maxVelocity(), config.maxAcceleration()));
  }

  @Override
  public void updateInputs(SingleRollerIOInputs inputs) {
    // DO NOT RUN WHEN DISABLED
    if (DriverStation.isDisabled()) {
      stop();
    } else if (isClosedLoop) {
      double feedforward = FeedforwardController.calculate(ff, controller.getSetpoint());
      double feedback = controller.calculate(sim.getAngularPositionRotations());
      setVolts(feedforward + feedback);
    }

    inputs.connected = true;
    sim.update(LoggedRobot.defaultPeriodSecs);

    inputs.position = Rotations.of(sim.getAngularPositionRotations());
    inputs.velocity = RPM.of(sim.getAngularVelocityRPM());

    inputs.appliedVoltage = sim.getInputVoltage();
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();

    inputs.positionGoal = Rotations.of(controller.getGoal().position);
    inputs.positionSetpoint = Rotations.of(controller.getSetpoint().position);
    inputs.velocitySetpoint = RotationsPerSecond.of(controller.getSetpoint().velocity);
  }

  private void setVolts(double volts) {
    double voltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(voltage);
  }

  @Override
  public void runVolts(double volts) {
    isClosedLoop = false;
    setVolts(volts);
  }

  @Override
  public void setGoal(Angle position) {
    isClosedLoop = true;
    controller.setGoal(new TrapezoidProfile.State(position.in(Rotations), 0));
  }

  @Override
  public void resetPosition(Angle position) {
    sim.setAngle(Units.rotationsToRadians(position.in(Rotations)));
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
