package org.robotzgarage.frc2026.subsystems.rollers.single;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LoggedRobot;
import org.robotzgarage.frc2026.util.MotorConfigs.GenericPIDConfig;

public interface SingleRollerIO {
  @AutoLog
  public static class SingleRollerIOInputs {
    public boolean connected = false;

    public Angle position = Rotations.zero();
    public AngularVelocity velocity = RotationsPerSecond.zero();

    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;

    public Angle positionGoal = Rotations.zero();
    public Angle positionSetpoint = Rotations.zero();
    public AngularVelocity velocitySetpoint = RotationsPerSecond.zero();
  }

  public default void updateInputs(SingleRollerIOInputs inputs) {}

  /** Run roller at set voltage */
  public default void runVolts(double volts) {}

  /** Run roller at specified percentage. Input should be -1.0 to 1.0 */
  public default void runPercentOutput(double percent) {}

  /** Run rollers with specified Amps */
  public default void runTorqueCurrent(Current current) {}

  /** Run rollers with specified Amps */
  public default void runVelocityTorqueCurrent(AngularVelocity velocity) {}

  /** Run roller at set Velocity */
  public default void setVelocity(AngularVelocity velocity) {}

  /** Set goal position of roller to position */
  public default void setGoal(Angle goal) {}

  /** Reset encoder to position */
  public default void resetPosition(Angle position) {}

  /** Stop roller */
  public default void stop() {}

  /** Gets the update frequency for simulation */
  public default double getLastTimestampDelta() {
    return LoggedRobot.defaultPeriodSecs;
  }

  public default GenericPIDConfig readPID() {
    return new GenericPIDConfig(0.0, 0.0, 0.0, 0.0);
  }

  public default void updatePID(double kP, double kI, double kD) {}
}
