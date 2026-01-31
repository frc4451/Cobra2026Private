package frc.robot.subsystems.superstructure.shooterComponent.shooterComponentConstan;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import org.robotzgarage.frc2026.subsystems.pivot.Pivot.PivotConfig;
import org.robotzgarage.frc2026.subsystems.pivot.PivotMechanism.PivotMechanismConfig;
import org.robotzgarage.frc2026.util.CTREUtils.MotionMagicHelpers;
import org.robotzgarage.frc2026.util.MechanismConfigs.RootMechanismConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfigSim;

public class HoodConstants {
  public static final double reduction = 5;

  public static final PivotConfig pivotConfig =
      PivotConfig.builder()
          .gearbox(DCMotor.getKrakenX60Foc(1))
          .reduction(reduction)
          .armLength(Feet.of(1))
          .initialAngle(Degrees.of(0))
          .minAngle(Degrees.of(0))
          .maxAngle(Degrees.of(50))
          .tolerance(Degrees.of(1))
          .simulateGravity(false)
          .moi(0.00002)
          .build();

  public static final TalonFXConfig hoodConfig =
      TalonFXConfig.builder()
          .canId(42)
          .canBus(Constants.rioBus)
          .reduction(reduction)
          .currentLimitAmps(30)
          .invert(true)
          .isBrakeMode(true)
          .foc(true)
          .slot0Config(
              // When in doubt, refer to CTRE
              // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html
              new Slot0Configs()
                  // feedforward
                  .withKS(MotionMagicHelpers.getVoltage(0.0))
                  .withGravityType(GravityTypeValue.Arm_Cosine)
                  // .withKG(0.04)
                  .withKV(MotionMagicHelpers.getRotationsVelocity(0.5))
                  // .withKA(MotionMagicHelpers.getRotationsAcceleration(0.12))
                  // feedback
                  .withKP(MotionMagicHelpers.getRotations(6.0))
                  // .withKI(MotionMagicHelpers.getRotationsVelocity(3.0))
                  .withKD(MotionMagicHelpers.getRotationsVelocity(0.25)))
          .mmConfig(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(MotionMagicHelpers.getRotationsVelocity(2))
                  .withMotionMagicAcceleration(MotionMagicHelpers.getRotationsAcceleration(1.0))
                  .withMotionMagicJerk(MotionMagicHelpers.getRotationsJerk(1.0)))
          .softwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitEnable(true)
                  .withForwardSoftLimitThreshold(
                      pivotConfig.maxAngle().minus(pivotConfig.tolerance()))
                  .withReverseSoftLimitEnable(true)
                  .withReverseSoftLimitThreshold(
                      pivotConfig.minAngle().plus(pivotConfig.tolerance())))
          .build();

  public static final TalonFXConfigSim hoodConfigSim =
      TalonFXConfigSim.builder()
          .config(hoodConfig)
          .dcMotor(pivotConfig.gearbox())
          .moi(pivotConfig.moi())
          .build();

  public static final RootMechanismConfig rootMechanismConfig =
      RootMechanismConfig.builder()
          .displayHeight(1.0)
          .displayWidth(1.0)
          .rootPosition(new Translation2d(0.5, 0.5))
          .build();

  public static final PivotMechanismConfig measuredPivotMechanismConfig =
      PivotMechanismConfig.builder()
          .name("Measured")
          .initialAngle(pivotConfig.initialAngle())
          .length(Units.inchesToMeters(12.0))
          .width(10)
          .color(new Color8Bit(Color.kDarkBlue))
          .build();

  public static final PivotMechanismConfig setpointPivotMechanismConfig =
      PivotMechanismConfig.builder()
          .name("Setpoint")
          .initialAngle(pivotConfig.initialAngle())
          .length(Units.inchesToMeters(12.0))
          .width(10)
          .color(new Color8Bit(Color.kSteelBlue))
          .build();

  public static final PivotMechanismConfig goalPivotMechanismConfig =
      PivotMechanismConfig.builder()
          .name("Goal")
          .initialAngle(pivotConfig.initialAngle())
          .length(Units.inchesToMeters(12.0))
          .width(10)
          .color(new Color8Bit(Color.kLightBlue))
          .build();
}
