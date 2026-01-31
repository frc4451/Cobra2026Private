package frc.robot.subsystems.prototyping;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import org.robotzgarage.frc2026.subsystems.flywheel.Flywheel.FlywheelConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfigFollower;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfigFollowerSim;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfigSim;

public class ShooterV1Constants {
  public static final double shooterReduction = 1.0;
  public static final double feederReduction = 4.0;
  public static final double kickerReduction = 4.0;

  public static final FlywheelConfig shooterFlywheelConfig =
      FlywheelConfig.builder()
          .radius(Inches.of(3.0))
          .gearbox(DCMotor.getKrakenX60Foc(1))
          .moi(0.00002)
          .reduction(shooterReduction)
          .build();

  public static final FlywheelConfig feederFlywheelConfig =
      FlywheelConfig.builder()
          .radius(Inches.of(3.0))
          .gearbox(DCMotor.getFalcon500Foc(1))
          .moi(0.00002)
          .reduction(feederReduction)
          .build();

  public static final FlywheelConfig kickerFlywheelConfig =
      FlywheelConfig.builder()
          .radius(Inches.of(3.0))
          .gearbox(DCMotor.getFalcon500Foc(1))
          .moi(0.00002)
          .reduction(kickerReduction)
          .build();

  public static final TalonFXConfig leaderShooterConfig =
      TalonFXConfig.builder()
          .canId(21)
          .canBus(Constants.rioBus)
          // .phoenixUpdateFreqHz(50)
          .reduction(shooterReduction)
          .currentLimitAmps(40)
          .invert(true)
          .isBrakeMode(true)
          .foc(Constants.focEnabled)
          .slot0Config(
              new Slot0Configs()
                  // feedforward
                  .withKS(0.05)
                  .withKV(0.05)
                  .withKA(0.0)
                  // feedback
                  .withKP(1.0)
                  .withKI(0.0)
                  .withKD(0.0))
          .mmConfig(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(5.0 * shooterReduction)
                  .withMotionMagicAcceleration(4.5 * shooterReduction)
                  .withMotionMagicJerk(20.0 * shooterReduction))
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

  //   public static final TalonFXConfig followerShooterConfig =
  //       TalonFXConfig.builder()
  //           .canId(22)
  //           .canBus(Constants.rioBus)
  //           // .phoenixUpdateFreqHz(50)
  //           .reduction(shooterReduction)
  //           .currentLimitAmps(30)
  //           .invert(true)
  //           .isBrakeMode(true)
  //           .foc(Constants.focEnabled)
  //           .slot0Config(
  //               new Slot0Configs()
  //                   // feedforward
  //                   .withKS(0.05)
  //                   .withKV(0.05)
  //                   .withKA(0.0)
  //                   // feedback
  //                   .withKP(1.0)
  //                   .withKI(0.0)
  //                   .withKD(0.0))
  //           .mmConfig(
  //               new MotionMagicConfigs()
  //                   .withMotionMagicCruiseVelocity(5.0 * shooterReduction)
  //                   .withMotionMagicAcceleration(4.5 * shooterReduction)
  //                   .withMotionMagicJerk(20.0 * shooterReduction))
  //           .build();

  public static final TalonFXConfig feederShooterConfig =
      TalonFXConfig.builder()
          .canId(23)
          .canBus(Constants.rioBus)
          // .phoenixUpdateFreqHz(50)
          .reduction(feederReduction)
          .currentLimitAmps(30)
          .invert(true)
          .isBrakeMode(true)
          .foc(Constants.focEnabled)
          .slot0Config(
              new Slot0Configs()
                  // feedforward
                  .withKS(0.05)
                  .withKV(0.05)
                  .withKA(0.0)
                  // feedback
                  .withKP(1.0)
                  .withKI(0.0)
                  .withKD(0.0))
          .mmConfig(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(5.0 * feederReduction)
                  .withMotionMagicAcceleration(4.5 * feederReduction)
                  .withMotionMagicJerk(20.0 * feederReduction))
          .build();

  public static final TalonFXConfigSim feederConfigSim =
      TalonFXConfigSim.builder()
          .config(feederShooterConfig)
          .dcMotor(feederFlywheelConfig.gearbox())
          .moi(feederFlywheelConfig.moi())
          .build();

  public static final TalonFXConfig kickerShooterConfig =
      TalonFXConfig.builder()
          .canId(24)
          .canBus(Constants.rioBus)
          // .phoenixUpdateFreqHz(50)
          .reduction(kickerReduction)
          .currentLimitAmps(30)
          .invert(true)
          .isBrakeMode(true)
          .foc(Constants.focEnabled)
          .slot0Config(
              new Slot0Configs()
                  // feedforward
                  .withKS(0.05)
                  .withKV(0.05)
                  .withKA(0.0)
                  // feedback
                  .withKP(1.0)
                  .withKI(0.0)
                  .withKD(0.0))
          .mmConfig(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(5.0 * kickerReduction)
                  .withMotionMagicAcceleration(4.5 * kickerReduction)
                  .withMotionMagicJerk(20.0 * kickerReduction))
          .build();

  //   public static final TalonFXConfigSim leaderConfigSim =
  //       TalonFXConfigSim.builder()
  //           .config(leaderShooterConfig)
  //           .dcMotor(shooterFlywheelConfig.gearbox())
  //           .moi(shooterFlywheelConfig.moi())
  //           .build();

  //   public static final TalonFXConfigSim followerConfigSim =
  //       TalonFXConfigSim.builder()
  //           .config(followerShooterConfig)
  //           .dcMotor(shooterFlywheelConfig.gearbox())
  //           .moi(shooterFlywheelConfig.moi())
  //           .build();

  public static final TalonFXConfigSim kickerConfigSim =
      TalonFXConfigSim.builder()
          .config(kickerShooterConfig)
          .dcMotor(kickerFlywheelConfig.gearbox())
          .moi(kickerFlywheelConfig.moi())
          .build();
}
