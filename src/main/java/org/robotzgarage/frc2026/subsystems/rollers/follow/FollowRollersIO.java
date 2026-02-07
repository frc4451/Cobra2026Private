package org.robotzgarage.frc2026.subsystems.rollers.follow;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;
import org.robotzgarage.frc2026.util.MotorConfigs.GenericPIDConfig;

public interface FollowRollersIO {
  @AutoLog
  public static class FollowRollersMagicIOInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;

    public Angle leaderPosition = Rotations.zero();
    public AngularVelocity leaderVelocity = RotationsPerSecond.zero();
    ;

    public double leaderAppliedVoltage = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
    public double leaderTorqueCurrentAmps = 0.0;
    public double leaderTemperatureCelsius = 0.0;

    public Angle followerPosition = Rotations.zero();
    public AngularVelocity followerVelocity = RotationsPerSecond.zero();
    ;

    public double followerAppliedVoltage = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerTorqueCurrentAmps = 0.0;
    public double followerTemperatureCelsius = 0.0;

    public Angle positionGoal = Rotations.zero();
    public Angle positionSetpoint = Rotations.zero();
    public AngularVelocity velocitySetpoint = RotationsPerSecond.zero();
  }

  public default void updateInputs(FollowRollersMagicIOInputs inputs) {}

  /** Run rollers at set voltage */
  public default void runVolts(double volts) {}

  /** Set goal position of rollers to position */
  public default void setGoal(Angle position) {}

  /** Reset encoders to position */
  public default void resetPosition(Angle position) {}

  /** Stop rollers */
  public default void stop() {}

  public default GenericPIDConfig readPID() {
    return new GenericPIDConfig(0, 0, 0, 0);
  }

  public default void updatePID(double kP, double kI, double kD) {}
}
