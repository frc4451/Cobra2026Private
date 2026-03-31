package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import org.robotzgarage.frc2026.subsystems.pivot.Pivot.PivotConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.CANCoderConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXSConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXSConfigSim;

public class HoodConstants {

  public static final double hoodReductionRotorToSensor = 1.0;
  public static final double hoodReductionSensorToMechanism = 1.0;

  public static final double hoodReduction =
      hoodReductionRotorToSensor * hoodReductionSensorToMechanism;

  public static final PivotConfig pivotConfig =
      PivotConfig.builder()
          .gearbox(DCMotor.getMinion(1))
          .reduction(hoodReduction)
          .armLength(Feet.of(1))
          .initialAngle(Degrees.of(28))
          .minAngle(Degrees.of(28))
          .maxAngle(Degrees.of(50))
          .tolerance(Degrees.of(1))
          .simulateGravity(false)
          .moi(0.00002)
          .build();

  public static final CANCoderConfig cancoderConfig =
      CANCoderConfig.builder()
          .canId(24)
          .canBus(Constants.susBus)
          .rotorToSensorReduction(hoodReductionRotorToSensor)
          .sensorToMechanismReduction(hoodReductionSensorToMechanism)
          .isClockwisePositive(true)
          // Set the Pivot to "DEPLOYED", keep this as Zero, then assign
          // the Absolute Position of the CANCoder to this magnetOffset
          //   .magnetOffset(Radians.of(0.75))
          .magnetOffset(Degrees.of(0))
          //   .calcOffset(Degrees.of(240))
          .discontinuity(0.875)
          //   .discontinuity(MathUtil.inputModulus(Units.degreesToRotations(90), 0, 1) / 2.0)
          //   .magnetOffset(Rotations.of(0.1728515625).plus(Rotations.of(0.668945)))
          //   .minAngle(Degrees.of(0.0))
          //   .maxAngle(Degrees.of(90.0))
          .build();

  public static final TalonFXSConfig hoodConfig =
      TalonFXSConfig.builder()
          .canId(23)
          .canBus(Constants.susBus)
          .phoenixUpdateFreqHz(50)
          .reduction(hoodReduction)
          .currentLimitAmps(40)
          .invert(false)
          .isBrakeMode(true)
          .foc(true)
          .slot0Config(
              // When in doubt, refer to CTRE
              // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html
              new Slot0Configs()
                  // feedforward
                  .withKS(0.05)
                  .withGravityType(GravityTypeValue.Arm_Cosine)
                  .withKG(0.0)
                  .withKV(0.0)
                  //   .withKV(MotionMagicHelpers.getRotationsVelocity(0.5))
                  // .withKA(MotionMagicHelpers.getRotationsAcceleration(0.12))
                  // feedback
                  .withKP(1.0)
                  // .withKI(MotionMagicHelpers.getRotationsVelocity(3.0))
                  .withKD(0.0))
          .mmConfig(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(100.0 * hoodReduction)
                  .withMotionMagicAcceleration(400.0 * hoodReduction)
                  .withMotionMagicJerk(1600.0 * hoodReduction))
          .build();

  public static final TalonFXSConfigSim hoodConfigSim =
      TalonFXSConfigSim.builder()
          .hardware(hoodConfig)
          .dcMotor(pivotConfig.gearbox())
          .moi(pivotConfig.moi())
          .build();
}
