package org.robotzgarage.frc2026.subsystems.flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import java.util.Optional;
import lombok.Builder;
import lombok.NonNull;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.subsystems.rollers.follow.FollowRollers;
import org.robotzgarage.frc2026.subsystems.rollers.follow.FollowRollersIO;

public class FlywheelFollower extends FollowRollers {
  @Builder
  public record FlywheelConfig(
      @NonNull DCMotor gearbox, @NonNull Distance radius, double moi, double reduction) {}

  private final Optional<FlywheelSim> flywheelSim;

  private Angle LeaderlastPosition = Rotations.zero();
  private Angle FollowerlastPosition = Rotations.zero();

  public FlywheelFollower(String name, FollowRollersIO io, FlywheelConfig flywheelconfig) {
    super(name, io);

    if (RobotBase.isSimulation()) {
      flywheelSim =
          Optional.of(
              new FlywheelSim(
                  LinearSystemId.createFlywheelSystem(
                      flywheelconfig.gearbox(), flywheelconfig.moi(), flywheelconfig.reduction()),
                  flywheelconfig.gearbox()));
    } else {
      flywheelSim = Optional.empty();
    }
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(
        name + "/Leader/PositionDeltaRotations",
        inputs.leaderPosition.minus(LeaderlastPosition).in(Rotations));
    Logger.recordOutput(
        name + "/Follower/PositionDeltaRotations",
        inputs.followerPosition.minus(FollowerlastPosition).in(Rotations));

    Logger.recordOutput(name + "/Leader/VelocityRPS", inputs.leaderVelocity.in(RotationsPerSecond));
    Logger.recordOutput(
        name + "/Follower/VelocityRPS", inputs.followerVelocity.in(RotationsPerSecond));
    Logger.recordOutput(
        name + "/VelocitySetpointRPS", inputs.velocitySetpoint.in(RotationsPerSecond));

    LeaderlastPosition = inputs.leaderPosition;
    FollowerlastPosition = inputs.followerPosition;

    // Flywheel Simulation, if present. SIM only tracks the leader motor
    flywheelSim.ifPresent(
        flywheelSim -> {
          flywheelSim.setInputVoltage(inputs.leaderAppliedVoltage);
          flywheelSim.setAngularVelocity((inputs.leaderVelocity.in(RadiansPerSecond)));
          // flywheelSim.update(io.getLastTimestampDelta());

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

  public void setVoltage(double voltage) {
    io.runVolts(voltage);
  }
}
