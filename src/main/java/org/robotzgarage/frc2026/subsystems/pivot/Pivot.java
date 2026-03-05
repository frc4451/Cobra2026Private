package org.robotzgarage.frc2026.subsystems.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.Optional;
import lombok.Builder;
import lombok.Getter;
import lombok.NonNull;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRoller;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;

public class Pivot extends SingleRoller {
  @Builder
  public record PivotConfig(
      @NonNull DCMotor gearbox,
      @NonNull Distance armLength,
      @NonNull Angle minAngle,
      @NonNull Angle maxAngle,
      Angle initialAngle,
      Angle tolerance,
      boolean simulateGravity,
      double moi,
      double reduction) {
    public PivotConfig {
      if (initialAngle == null) {
        initialAngle = Rotations.zero();
      }

      if (tolerance == null) {
        tolerance = Rotations.zero();
      }
    }
  }

  @Getter private final PivotConfig config;

  @Getter private final String name;

  private final Optional<SingleJointedArmSim> pivotSim;

  public Pivot(String name, SingleRollerIO io, PivotConfig config) {
    super(name, io);

    // tolerance = config.tolerance();
    this.name = name;
    this.config = config;

    // this.io.resetPosition(config.initialAngle());

    if (RobotBase.isSimulation()) {
      pivotSim =
          Optional.of(
              new SingleJointedArmSim(
                  LinearSystemId.createSingleJointedArmSystem(
                      config.gearbox(), config.moi(), config.reduction()),
                  config.gearbox(),
                  config.reduction(),
                  config.armLength().in(Meters),
                  config.minAngle().in(Radians),
                  config.maxAngle().in(Radians),
                  config.simulateGravity(),
                  config.initialAngle().in(Radians)));
    } else {
      pivotSim = Optional.empty();
    }
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(name + "/CurrentAngleDegrees", this.getPosition().in(Degrees));
    Logger.recordOutput(name + "/GoalAngleDegrees", this.getGoalPosition().in(Degrees));
    Logger.recordOutput(name + "/VelocityRPS", this.getVelocity().in(RotationsPerSecond));

    // Pivot Simulation, if present
    pivotSim.ifPresent(
        pivotSim -> {
          pivotSim.setInputVoltage(inputs.appliedVoltage);
          // pivotSim.update(io.getLastTimestampDelta());

          String logRoot = name + "/PhysicalSim";
          Logger.recordOutput(logRoot + "/CurrentDrawAmps", pivotSim.getCurrentDrawAmps());
          Logger.recordOutput(logRoot + "/AngleRads", pivotSim.getAngleRads());
          Logger.recordOutput(logRoot + "/VelocityRadsPerSec", pivotSim.getVelocityRadPerSec());
          Logger.recordOutput(logRoot + "/HasHitLowerLimit", pivotSim.hasHitLowerLimit());
          Logger.recordOutput(logRoot + "/HasHitUpperLimit", pivotSim.hasHitUpperLimit());
        });
  }

  /**
   * Inputs are clamped to `config.minAngle` and `config.maxAngle`
   *
   * @param targetPosition - desired `Angle` to profile
   */
  public void setGoalPosition(Angle targetPosition) {
    io.setGoal(targetPosition);
    // io.setGoal(
    //     Rotations.of(
    //         MathUtil.clamp(
    //             targetPosition.in(Rotations),
    //             config.minAngle().in(Rotations),
    //             config.maxAngle().in(Rotations))));
  }

  public Angle getPosition() {
    return inputs.position;
  }

  public Angle getPositionSetpoint() {
    return inputs.positionSetpoint;
  }

  public Angle getGoalPosition() {
    return inputs.positionGoal;
  }

  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }

  public boolean isNear(Angle angle) {
    return inputs.position.isNear(angle, config.tolerance());
  }

  public boolean isNear(Angle angle, Angle tolerance) {
    return inputs.position.isNear(angle, tolerance);
  }

  public boolean isNearLowerLimit() {
    return isNear(config.minAngle());
  }

  public boolean isNearUpperLimit() {
    return isNear(config.maxAngle());
  }

  public boolean atGoal() {
    return isNear(getGoalPosition());
  }
}
