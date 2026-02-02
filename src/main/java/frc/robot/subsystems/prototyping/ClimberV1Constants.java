package frc.robot.subsystems.prototyping;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import org.robotzgarage.frc2026.subsystems.flywheel.Flywheel.FlywheelConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfigSim;

public class ClimberV1Constants {
  public static final double climberReduction = 5.0;

  public static final FlywheelConfig flywheelConfig =
      FlywheelConfig.builder()
          .radius(Inches.of(3.0))
          .gearbox(DCMotor.getKrakenX60Foc(1))
          .moi(0.00002)
          .reduction(climberReduction)
          .build();

  public static final TalonFXConfig climberConfig =
      TalonFXConfig.builder()
          .canId(51)
          .canBus(Constants.rioBus)
          // .phoenixUpdateFreqHz(50)
          .reduction(climberReduction)
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
                  .withKP(10.0)
                  .withKI(0.0)
                  .withKD(0.0))
          .mmConfig(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(5.0 * climberReduction)
                  .withMotionMagicAcceleration(4.5 * climberReduction)
                  .withMotionMagicJerk(20.0 * climberReduction))
          .build();

  public static final TalonFXConfigSim climberConfigSim =
      TalonFXConfigSim.builder()
          .config(climberConfig)
          .dcMotor(flywheelConfig.gearbox())
          .moi(flywheelConfig.moi())
          .build();
}
