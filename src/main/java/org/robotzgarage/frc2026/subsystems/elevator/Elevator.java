package org.robotzgarage.frc2026.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.Optional;
import lombok.Builder;
import lombok.Getter;
import lombok.NonNull;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRoller;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;

public class Elevator extends SingleRoller {
  @Builder
  public record ElevatorConfig(
      @NonNull DCMotor gearbox,
      @NonNull Distance minHeight,
      @NonNull Distance maxHeight,
      @NonNull Distance drumRadius,
      Distance initialHeight,
      Distance tolerance,
      Mass carriageMass,
      boolean simulateGravity,
      double moi,
      double reduction) {

    public ElevatorConfig {
      if (initialHeight == null) {
        initialHeight = Inches.zero();
      }

      if (tolerance == null) {
        tolerance = Inches.zero();
      }
    }

    /**
     * Calculates circumference of Elevator drum to help determine distances from rotor position
     *
     * @return 2 * PI * drumRadius
     */
    public Distance drumCircumference() {
      return drumRadius().times(2 * Math.PI);
    }

    public Distance minHeightCompensated() {
      return minHeight().times(drumCircumference().magnitude());
    }

    public Distance maxHeightCompensated() {
      return maxHeight().times(drumCircumference().magnitude());
    }
  }

  @Getter private final ElevatorConfig config;

  private final Optional<ElevatorSim> elevatorSim;

  public Elevator(String name, SingleRollerIO io, ElevatorConfig config) {
    super(name, io);
    this.config = config;

    if (RobotBase.isSimulation()) {
      elevatorSim =
          Optional.of(
              new ElevatorSim(
                  LinearSystemId.createElevatorSystem(
                      config.gearbox(),
                      config.carriageMass().in(Kilograms),
                      config.drumRadius().in(Meters),
                      config.reduction()),
                  config.gearbox(),
                  config.minHeight().in(Meters),
                  config.maxHeight().in(Meters),
                  config.simulateGravity(),
                  config.initialHeight().in(Meters)));
    } else {
      elevatorSim = Optional.empty();
    }

    this.setHeight(config.initialHeight());
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(name + "/CurrentHeightInches", this.getHeight().in(Inches));
    Logger.recordOutput(name + "/GoalHeightInches", this.getGoalHeight().in(Inches));
    Logger.recordOutput(name + "/AtGoal", this.atGoal());
    Logger.recordOutput(name + "/VelocityInchesPerSecond", this.getVelocity().in(InchesPerSecond));

    // Pivot Simulation, if present
    elevatorSim.ifPresent(
        elevatorSim -> {
          elevatorSim.setInputVoltage(inputs.appliedVoltage);
          RoboRioSim.setVInVoltage(
              BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
          elevatorSim.update(io.getLastTimestampDelta());

          String logRoot = name + "/PhysicalSim";
          Logger.recordOutput(logRoot + "/CurrentDrawAmps", elevatorSim.getCurrentDrawAmps());
          Logger.recordOutput(
              logRoot + "/PositionInches", Units.metersToInches(elevatorSim.getPositionMeters()));
          Logger.recordOutput(
              logRoot + "/VelocityInchesPerSec",
              Units.metersToInches(elevatorSim.getVelocityMetersPerSecond()) / config.reduction());
          Logger.recordOutput(logRoot + "/HasHitLowerLimit", elevatorSim.hasHitLowerLimit());
          Logger.recordOutput(logRoot + "/HasHitUpperLimit", elevatorSim.hasHitUpperLimit());
        });
  }

  public LinearVelocity getVelocity() {
    return InchesPerSecond.of(
        inputs.velocity.in(RotationsPerSecond) * config.drumCircumference().in(Inches));
  }

  public Distance getHeight() {
    return Inches.of(inputs.position.in(Rotations) * config.drumCircumference().in(Inches));
  }

  public Distance getGoalHeight() {
    return Inches.of(inputs.positionGoal.in(Rotations) * config.drumCircumference().in(Inches));
  }

  public Distance getSetpointHeight() {
    return Inches.of(inputs.positionSetpoint.in(Rotations) * config.drumCircumference().in(Inches));
  }

  public void setHeight(Distance position) {
    io.resetPosition(getClampedPosition(position));
  }

  public void setGoalHeight(Distance position) {
    io.setGoal(getClampedPosition(position));
  }

  public boolean isNear(Distance height) {
    return MathUtil.isNear(
        getHeight().in(Inches), height.in(Inches), config.tolerance().in(Inches));
  }

  public boolean isNearLowerLimit() {
    return isNear(config.minHeight());
  }

  public boolean isNearUpperLimit() {
    return isNear(config.maxHeight());
  }

  public boolean atGoal() {
    return isNear(
        Inches.of(inputs.positionGoal.in(Rotations) * config.drumCircumference().in(Inches)));
  }

  /**
   * Handle clamping desired positions such that the elevator can't slam the top or bottom of the
   * mechanism.
   *
   * @param desired `Distance` measurement that you want to reach
   * @return clamped angle for rotor
   */
  private Angle getClampedPosition(Distance desired) {
    double desiredPosition = desired.in(Inches) / config.drumCircumference().in(Inches);
    double clampedPosition =
        MathUtil.clamp(
            desiredPosition,
            config.minHeight().div(config.drumCircumference().magnitude()).in(Inches),
            config.maxHeight().div(config.drumCircumference().magnitude()).in(Inches));

    return Rotations.of(clampedPosition);
  }
}
