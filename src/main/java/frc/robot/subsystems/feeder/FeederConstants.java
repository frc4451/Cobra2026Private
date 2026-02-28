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
  public static final double spindexerReduction = 4.0;

  public static final FlywheelConfig towerFlywheelConfig =
      FlywheelConfig.builder()
          .radius(Inches.of(3.0))
          .gearbox(DCMotor.getFalcon500Foc(1))
          .moi(0.00002)
          .reduction(towerReduction)
          .build();

  public static final FlywheelConfig spindexerFlywheelConfig =
      FlywheelConfig.builder()
          .radius(Inches.of(3.0))
          .gearbox(DCMotor.getFalcon500Foc(1))
          .moi(0.00002)
          .reduction(spindexerReduction)
          .build();

  public static final TalonFXConfig towerConfig =
      TalonFXConfig.builder()
          .canId(41)
          .canBus(Constants.susBus)
          // .phoenixUpdateFreqHz(50)
          .reduction(towerReduction)
          .currentLimitAmps(30)
          .invert(true)
          .isBrakeMode(true)
          .foc(Constants.focEnabled)
          .slot0Config(
              new Slot0Configs()
                  // feedforward
                  .withKS( 0.008815)
                  .withKV(0.471267)
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

  public static final TalonFXConfig spindexerConfig =
      TalonFXConfig.builder()
          .canId(42)
          .canBus(Constants.susBus)
          // .phoenixUpdateFreqHz(50)
          .reduction(spindexerReduction)
          .currentLimitAmps(30)
          .invert(true)
          .isBrakeMode(true)
          .foc(Constants.focEnabled)
          .slot0Config(
              new Slot0Configs()
                  // feedforward
                  .withKS(0.008331)
                  .withKV(0.471394)
                  .withKA(0.0)
                  // feedback
                  .withKP(1.0)
                  .withKI(0.0)
                  .withKD(0.0))
          .mmConfig(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(5.0 * spindexerReduction)
                  .withMotionMagicAcceleration(4.5 * spindexerReduction)
                  .withMotionMagicJerk(20.0 * spindexerReduction))
          .build();

  public static final TalonFXConfigSim spindexerConfigSim =
      TalonFXConfigSim.builder()
          .config(spindexerConfig)
          .dcMotor(spindexerFlywheelConfig.gearbox())
          .moi(spindexerFlywheelConfig.moi())
          .build();
}
