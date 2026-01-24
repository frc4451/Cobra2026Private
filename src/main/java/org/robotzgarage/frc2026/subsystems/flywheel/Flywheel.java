package org.robotzgarage.frc2026.subsystems.flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import java.util.Optional;
import lombok.Builder;
import lombok.NonNull;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRoller;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;

public class Flywheel extends SingleRoller {
  @Builder
  public record FlywheelConfig(
      @NonNull DCMotor gearbox, @NonNull Distance radius, double moi, double reduction) {}

  private final Optional<FlywheelSim> flywheelSim;

  private Angle lastPosition = Rotations.zero();

  public Flywheel(String name, SingleRollerIO io, FlywheelConfig config) {
    super(name, io);

    if (RobotBase.isSimulation()) {
      flywheelSim =
          Optional.of(
              new FlywheelSim(
                  LinearSystemId.createFlywheelSystem(
                      config.gearbox(), config.moi(), config.reduction()),
                  config.gearbox()));
    } else {
      flywheelSim = Optional.empty();
    }
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(
        name + "/PositionDeltaRotations", inputs.position.minus(lastPosition).in(Rotations));

    Logger.recordOutput(name + "/VelocityRPS", inputs.velocity.in(RotationsPerSecond));
    Logger.recordOutput(
        name + "/VelocitySetpointRPS", inputs.velocitySetpoint.in(RotationsPerSecond));

    lastPosition = inputs.position;

    // Flywheel Simulation, if present
    flywheelSim.ifPresent(
        flywheelSim -> {
          flywheelSim.setInputVoltage(inputs.appliedVoltage);
          flywheelSim.setAngularVelocity((inputs.velocity.in(RadiansPerSecond)));
          flywheelSim.update(io.getLastTimestampDelta());

          String logRoot = name + "/PhysicalSim";
          Logger.recordOutput(
              logRoot + "/VelocityRotPerSec",
              flywheelSim.getAngularVelocity().in(RotationsPerSecond));
          Logger.recordOutput(
              logRoot + "/OmegaRotPerSecSq",
              flywheelSim.getAngularAcceleration().in(RotationsPerSecondPerSecond));
          Logger.recordOutput(logRoot + "/TorqueNwtMeters", flywheelSim.getTorqueNewtonMeters());
          Logger.recordOutput(logRoot + "/CurrentDrawAmps", flywheelSim.getCurrentDrawAmps());
          Logger.recordOutput(logRoot + "/InputVoltage", flywheelSim.getInputVoltage());
        });
  }

  public void setVelocity(AngularVelocity velocity) {
    io.setVelocity(velocity);
  }

  public void setPercentOutput(double percent) {
    io.runPercentOutput(percent);
  }

  public void setVoltage(double voltage) {
    io.runVolts(voltage);
  }
}
