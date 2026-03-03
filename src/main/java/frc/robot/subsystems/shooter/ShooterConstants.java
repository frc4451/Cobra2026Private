package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import org.robotzgarage.frc2026.subsystems.flywheel.FlywheelFollower.FlywheelConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfigFollower;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfigFollowerSim;

public class ShooterConstants {
  public static final double shooterReduction = 22.0 / 24.0;

  public static final FlywheelConfig shooterFlywheelConfig =
      FlywheelConfig.builder()
          .radius(Inches.of(3.0))
          .gearbox(DCMotor.getKrakenX60Foc(1))
          .moi(0.00002)
          .reduction(shooterReduction)
          .build();

  public static final TalonFXConfig leaderShooterConfig =
      TalonFXConfig.builder()
          .canId(21)
          .canBus(Constants.susBus)
          // .phoenixUpdateFreqHz(50)
          .reduction(shooterReduction)
          .currentLimitAmps(40)
          .invert(false)
          .isBrakeMode(false)
          .foc(Constants.focEnabled)
          .slot0Config(
              new Slot0Configs()
                  // feedforward
                  .withKS(20.0)
                  .withKV(0.2)
                  .withKA(0.0)
                  // feedback
                  .withKP(85.0)
                  .withKI(0.0)
                  .withKD(0.0))
          .mmConfig(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(5.0 * shooterReduction)
                  .withMotionMagicAcceleration(4.5 * shooterReduction)
                  .withMotionMagicJerk(20.0 * shooterReduction))
          .currentLimitConfig(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(120.0)
                  .withStatorCurrentLimitEnable(true)
              // .withSupplyCurrentLimit(null)
              )
          .torqueCurrentConfigs(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(100.0)
                  .withPeakReverseTorqueCurrent(-100.0))
          .build();

  public static final TalonFXConfigFollower flywheelFollowersConfig =
      TalonFXConfigFollower.builder()
          .leader(leaderShooterConfig)
          .followerCanId(22)
          .invertFollower(false)
          .build();

  public static final TalonFXConfigFollowerSim flywheelFollowersConfigSim =
      TalonFXConfigFollowerSim.builder()
          .hardware(flywheelFollowersConfig)
          .dcMotor(DCMotor.getKrakenX60Foc(2))
          .moi(0.002)
          .build();
}
