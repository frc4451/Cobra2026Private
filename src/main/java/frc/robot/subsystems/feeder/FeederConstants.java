package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import org.robotzgarage.frc2026.subsystems.flywheel.Flywheel.FlywheelConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfigSim;

public class FeederConstants {
  public static final double towerReduction = 4.0;
  public static final double kickerReduction = 4.0;

  public static final FlywheelConfig towerFlywheelConfig =
      FlywheelConfig.builder()
          .radius(Inches.of(3.0))
          .gearbox(DCMotor.getFalcon500Foc(1))
          .moi(0.00002)
          .reduction(towerReduction)
          .build();

  public static final FlywheelConfig kickerFlywheelConfig =
      FlywheelConfig.builder()
          .radius(Inches.of(3.0))
          .gearbox(DCMotor.getFalcon500Foc(1))
          .moi(0.00002)
          .reduction(kickerReduction)
          .build();

  public static final TalonFXConfig towerConfig =
      TalonFXConfig.builder()
          .canId(23)
          .canBus(Constants.rioBus)
          // .phoenixUpdateFreqHz(50)
          .reduction(towerReduction)
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
                  .withMotionMagicCruiseVelocity(5.0 * towerReduction)
                  .withMotionMagicAcceleration(4.5 * towerReduction)
                  .withMotionMagicJerk(20.0 * towerReduction))
          .build();

  public static final TalonFXConfigSim towerConfigSim =
      TalonFXConfigSim.builder()
          .config(towerConfig)
          .dcMotor(towerFlywheelConfig.gearbox())
          .moi(towerFlywheelConfig.moi())
          .build();

  public static final TalonFXConfig kickerConfig =
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

  public static final TalonFXConfigSim kickerConfigSim =
      TalonFXConfigSim.builder()
          .config(kickerConfig)
          .dcMotor(kickerFlywheelConfig.gearbox())
          .moi(kickerFlywheelConfig.moi())
          .build();
}
