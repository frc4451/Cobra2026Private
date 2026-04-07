package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import org.robotzgarage.frc2026.subsystems.flywheel.Flywheel.FlywheelConfig;
import org.robotzgarage.frc2026.subsystems.pivot.Pivot.PivotConfig;
import org.robotzgarage.frc2026.subsystems.pivot.PivotMechanism.PivotMechanismConfig;
import org.robotzgarage.frc2026.util.MechanismConfigs.RootMechanismConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.CANCoderConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfigSim;

public class IntakeConstants {
  public static final double intakeReduction = 1.0;

  public static final Angle zeroOffset = Degrees.of(120);

  public static final double endeffectorReductionRotorToSensor = (5.0 * 4.0);
  public static final double endeffectorReductionSensorToMechanism = 3.0;

  public static final double endeffectorReduction =
      endeffectorReductionRotorToSensor * endeffectorReductionSensorToMechanism;

  public static final Current rollerSupplyCurrentLimit = Amps.of(40.0);
  public static final Current rollerStatorCurrentLimit = Amps.of(80.0);

  public static final Current endeffectorSupplyCurrentLimit = Amps.of(40.0);
  public static final Current endeffectorStatorCurrentLimit = Amps.of(80.0);

  public static final FlywheelConfig flywheelConfig =
      FlywheelConfig.builder()
          .radius(Inches.of(3.0))
          .gearbox(DCMotor.getFalcon500Foc(1))
          .moi(0.00002)
          .reduction(intakeReduction)
          .build();

  public static final TalonFXConfig intakeConfig =
      TalonFXConfig.builder()
          .canId(31)
          .canBus(Constants.susBus)
          .phoenixUpdateFreqHz(50)
          .reduction(intakeReduction)
          .currentLimitAmps(40)
          .currentLimitConfig(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(rollerStatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(rollerSupplyCurrentLimit)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLowerLimit(rollerSupplyCurrentLimit)
                  .withSupplyCurrentLowerTime(0.1))
          .torqueCurrentConfigs(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(rollerStatorCurrentLimit)
                  .withPeakReverseTorqueCurrent(rollerStatorCurrentLimit.unaryMinus()))
          .invert(false)
          .isBrakeMode(true)
          .foc(true)
          .slot0Config(
              new Slot0Configs()
                  // feedforward
                  .withKS(0.138729)
                  .withKV(0.112423)
                  .withKA(0.0)
                  // feedback
                  .withKP(3.0)
                  .withKI(0.0)
                  .withKD(0.0))
          .mmConfig(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(5.0)
                  .withMotionMagicAcceleration(20)
                  .withMotionMagicJerk(80.0))
          .build();

  public static final TalonFXConfigSim intakeConfigSim =
      TalonFXConfigSim.builder()
          .config(intakeConfig)
          .dcMotor(flywheelConfig.gearbox())
          .moi(flywheelConfig.moi())
          .build();

  public static final PivotConfig pivotConfig =
      PivotConfig.builder()
          .gearbox(DCMotor.getFalcon500Foc(1))
          .reduction(endeffectorReduction)
          .armLength(Feet.of(1))
          .initialAngle(Degrees.of(90))
          .minAngle(Degrees.of(-2.0))
          .maxAngle(Degrees.of(90))
          .tolerance(Degrees.of(1))
          .simulateGravity(false)
          .moi(0.00002)
          .build();

  public static final CANCoderConfig cancoderConfig =
      CANCoderConfig.builder()
          .canId(33)
          .canBus(Constants.susBus)
          .rotorToSensorReduction(endeffectorReductionRotorToSensor)
          .sensorToMechanismReduction(endeffectorReductionSensorToMechanism)
          .isClockwisePositive(true)
          // Set the Pivot to "DEPLOYED", keep this as Zero, then assign
          // the Absolute Position of the CANCoder to this magnetOffset
          //   .magnetOffset(Radians.of(0.75))
          .magnetOffset(Degrees.of(92.5))
          .discontinuity(0.875)
          //   .discontinuity(MathUtil.inputModulus(Units.degreesToRotations(90), 0, 1) / 2.0)
          //   .magnetOffset(Rotations.of(0.1728515625).plus(Rotations.of(0.668945)))
          //   .minAngle(Degrees.of(0.0))
          //   .maxAngle(Degrees.of(90.0))
          .build();

  public static final TalonFXConfig endeffectorConfig =
      TalonFXConfig.builder()
          .canId(32)
          .canBus(Constants.susBus)
          .phoenixUpdateFreqHz(50)
          .reduction(endeffectorReduction)
          .currentLimitConfig(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(endeffectorStatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(endeffectorSupplyCurrentLimit)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLowerLimit(endeffectorSupplyCurrentLimit)
                  .withSupplyCurrentLowerTime(0.25))
          .torqueCurrentConfigs(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(endeffectorStatorCurrentLimit)
                  .withPeakReverseTorqueCurrent(endeffectorStatorCurrentLimit.unaryMinus()))
          .invert(false)
          .isBrakeMode(true)
          .foc(true)
          .slot0Config(
              // When in doubt, refer to CTRE
              // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html
              new Slot0Configs()
                  // feedforward
                  .withKS(0.75)
                  .withGravityType(GravityTypeValue.Arm_Cosine)
                  .withKG(0.0)
                  .withKV(0.0)
                  //   .withKV(MotionMagicHelpers.getRotationsVelocity(0.5))
                  // .withKA(MotionMagicHelpers.getRotationsAcceleration(0.12))
                  // feedback
                  .withKP(50.0)
                  // .withKI(MotionMagicHelpers.getRotationsVelocity(3.0))
                  .withKD(0.05))
          .mmConfig(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(600.0 * endeffectorReduction)
                  .withMotionMagicAcceleration(2400.0 * endeffectorReduction)
                  .withMotionMagicJerk(9600.0 * endeffectorReduction))
          //   .softwareLimitSwitch(
          //       new SoftwareLimitSwitchConfigs()
          //           .withForwardSoftLimitEnable(true)
          //           .withForwardSoftLimitThreshold(
          //               pivotConfig.maxAngle().minus(pivotConfig.tolerance()))
          //           .withReverseSoftLimitEnable(true)
          //           .withReverseSoftLimitThreshold(
          //               pivotConfig.minAngle().plus(pivotConfig.tolerance())))
          .build();

  public static final TalonFXConfigSim endeffectorConfigSim =
      TalonFXConfigSim.builder()
          .config(endeffectorConfig)
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
          .color(new Color8Bit(Color.kGreen))
          .build();

  public static final PivotMechanismConfig setpointPivotMechanismConfig =
      PivotMechanismConfig.builder()
          .name("Setpoint")
          .initialAngle(pivotConfig.initialAngle())
          .length(Units.inchesToMeters(12.0))
          .width(10)
          .color(new Color8Bit(Color.kLawnGreen))
          .build();

  public static final PivotMechanismConfig goalPivotMechanismConfig =
      PivotMechanismConfig.builder()
          .name("Goal")
          .initialAngle(pivotConfig.initialAngle())
          .length(Units.inchesToMeters(12.0))
          .width(10)
          .color(new Color8Bit(Color.kLightGreen))
          .build();
}
