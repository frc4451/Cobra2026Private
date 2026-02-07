package org.robotzgarage.frc2026.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.thethriftybot.devices.ThriftyNova.EncoderType;
import com.thethriftybot.util.Conversion.PositionUnit;
import com.thethriftybot.util.Conversion.VelocityUnit;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import lombok.Builder;
import lombok.NonNull;
import org.robotzgarage.frc2026.subsystems.rollers.feedforward_controller.FeedforwardController;

public class MotorConfigs {
  @Builder
  public record SimMotorConfig(
      @NonNull DCMotor motorModel,
      double reduction,
      double moi,
      double maxVelocity,
      double maxAcceleration,
      double kP,
      double kI,
      double kD,
      FeedforwardController ff) {}

  @Builder
  public record SimMotorConfigFollower(@NonNull SimMotorConfig leader, boolean invertFollower) {}

  @Builder
  public record TalonFXConfig(
      @NonNull Integer canId,
      @NonNull CANBus canBus,
      double reduction,
      double currentLimitAmps,
      boolean invert,
      boolean isBrakeMode,
      boolean foc,
      double phoenixUpdateFreqHz,
      Slot0Configs slot0Config,
      MotionMagicConfigs mmConfig,
      SoftwareLimitSwitchConfigs softwareLimitSwitch,
      CurrentLimitsConfigs currentLimitConfig,
      TorqueCurrentConfigs torqueCurrentConfigs) {
    public TalonFXConfig {

      // Default should be 50hz
      phoenixUpdateFreqHz = MathUtil.clamp(phoenixUpdateFreqHz, 50.0, 1000);

      // Safety should max out at 40A
      currentLimitAmps = Math.max(currentLimitAmps, 40.0);

      // Not all motors need soft limits, if not needed, provide a default
      if (softwareLimitSwitch == null) {
        softwareLimitSwitch =
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(false)
                .withReverseSoftLimitEnable(false);
      }

      if (currentLimitConfig == null) {
        currentLimitConfig =
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(currentLimitAmps);
      }

      if (torqueCurrentConfigs == null) {
        torqueCurrentConfigs = new TorqueCurrentConfigs();
      }
    }
  }

  @Builder
  public record TalonFXConfigSim(
      @NonNull TalonFXConfig config, @NonNull DCMotor dcMotor, double moi) {}

  @Builder
  public record TalonFXConfigFollower(
      @NonNull TalonFXConfig leader, @NonNull Integer followerCanId, boolean invertFollower) {}

  @Builder
  public record TalonFXConfigFollowerSim(
      @NonNull TalonFXConfigFollower hardware,
      @NonNull DCMotor dcMotor,
      double moi,
      boolean invertFollower) {}

  @Builder
  public record TalonFXSConfig(
      @NonNull Integer canId,
      @NonNull CANBus canBus,
      double reduction,
      double currentLimitAmps,
      boolean invert,
      boolean isBrakeMode,
      boolean foc,
      double phoenixUpdateFreqHz,
      Slot0Configs slot0Config,
      MotionMagicConfigs mmConfig,
      SoftwareLimitSwitchConfigs softwareLimitSwitch,
      MotorArrangementValue motorJST) {
    public TalonFXSConfig {
      // Default should be 50hz
      phoenixUpdateFreqHz = MathUtil.clamp(phoenixUpdateFreqHz, 50.0, 250);

      // Safety should max out at 40A
      currentLimitAmps = Math.max(currentLimitAmps, 40.0);

      // Not all motors need soft limits, if not needed, provide a default
      if (softwareLimitSwitch == null) {
        softwareLimitSwitch =
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(false)
                .withReverseSoftLimitEnable(false);
      }

      // It's assumed that if we don't have a motor preselected,
      // it is likely the Minion with the JST connector
      if (motorJST == null) {
        motorJST = MotorArrangementValue.Minion_JST;
      }
    }
  }

  @Builder
  public record TalonFXSConfigSim(
      @NonNull TalonFXSConfig hardware, @NonNull DCMotor dcMotor, double moi) {}

  /** Shorthand PID config for any generic PID controller */
  @Builder
  public record GenericPIDConfig(double p, double i, double d, double ff) {}

  @Builder
  public record ThriftyNovaConfig(
      @NonNull Integer canId,
      double reduction,
      double currentLimitAmpsSupply,
      double currentLimitAmpsStator,
      double rampRateUp,
      double rampRateDown,
      double maxOutput,
      boolean invert,
      boolean isBrakeMode,
      boolean enableSoftLimits,
      GenericPIDConfig pidConfig,
      PositionUnit positionUnit,
      VelocityUnit velocityUnit,
      EncoderType encoderType) {
    public ThriftyNovaConfig {
      // Assume that max current should be 40A per documentation
      currentLimitAmpsStator = Math.max(currentLimitAmpsStator, 40.0);
      currentLimitAmpsSupply = Math.max(currentLimitAmpsSupply, 40.0);

      // Default should be ROTATIONS to stay consistent with CTR-E logic
      if (positionUnit == null) {
        positionUnit = PositionUnit.ROTATIONS;
      }

      // Default should be ROTATIONS/SEC to stay consistent with CTR-E logic
      if (velocityUnit == null) {
        velocityUnit = VelocityUnit.ROTATIONS_PER_SEC;
      }

      // Assume we use the internal encoder if not specified otherwise
      if (encoderType == null) {
        encoderType = EncoderType.INTERNAL;
      }

      if (pidConfig == null) {
        pidConfig = GenericPIDConfig.builder().p(1).i(0).d(0).ff(0).build();
      }
    }
  }
}
