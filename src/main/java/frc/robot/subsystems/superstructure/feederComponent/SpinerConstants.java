package frc.robot.subsystems.superstructure.feederComponent;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import org.robotzgarage.frc2026.subsystems.flywheel.Flywheel.FlywheelConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfigSim;

public class SpinerConstants {
  public static final double reduction = 1.0;

  public static final FlywheelConfig flywheelConfig =
      FlywheelConfig.builder()
          .radius(Inches.of(3.0))
          .gearbox(DCMotor.getFalcon500Foc(1))
          .moi(0.00002)
          .reduction(reduction)
          .build();

  public static final TalonFXConfig spinerConfig =
      TalonFXConfig.builder()
          .canId(31)
          .canBus(Constants.rioBus)
          // .phoenixUpdateFreqHz(50)
          .reduction(reduction)
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
                  .withMotionMagicCruiseVelocity(5.0 * reduction)
                  .withMotionMagicAcceleration(4.5 * reduction)
                  .withMotionMagicJerk(20.0 * reduction))
          .build();

  public static final TalonFXConfigSim spinerConfigSim =
      TalonFXConfigSim.builder()
          .config(spinerConfig)
          .dcMotor(flywheelConfig.gearbox())
          .moi(flywheelConfig.moi())
          .build();
}
