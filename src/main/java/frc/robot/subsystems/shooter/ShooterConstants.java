package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;
import org.robotzgarage.frc2026.subsystems.flywheel.FlywheelFollower.FlywheelConfig;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.RobotRelativeTransform;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfigFollower;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfigFollowerSim;

public class ShooterConstants {
  public static final double shooterReduction = 22.0 / 24.0;

  public static final AngularVelocity maximumShooterVelocity = RotationsPerSecond.of(80);
  public static final AngularAcceleration maximumShooterAcceleration =
      RotationsPerSecondPerSecond.of(400);
  public static final Current supplyCurrentLimit = Amps.of(40.0);
  public static final Current statorCurrentLimit = Amps.of(100.0);

  // Collected from the center of the fuel from Cobra Week 1 CAD
  public static final Transform2d robotToShooter =
      RobotRelativeTransform.builder()
          .forwardPositive(Inches.of(4.995397))
          .leftPositive(Inches.of(-6.300))
          .upPositive(Inches.of(13.991715))
          .rollCCWPositive(Degrees.zero())
          .pitchCCWPositive(Degrees.zero())
          .yawCCWPositive(Degrees.zero())
          .build()
          .getTransform2d();

  public static final FlywheelConfig shooterFlywheelConfig =
      FlywheelConfig.builder()
          .radius(Inches.of(1.5))
          .gearbox(DCMotor.getKrakenX60Foc(2))
          .moi(0.002)
          .reduction(shooterReduction)
          .build();

  public static final TalonFXConfig leaderShooterConfig =
      TalonFXConfig.builder()
          .canId(21)
          .canBus(Constants.susBus)
          // .phoenixUpdateFreqHz(50)
          .reduction(shooterReduction)
          .currentLimitAmps(supplyCurrentLimit.in(Amps))
          .invert(false)
          .isBrakeMode(false)
          .foc(Constants.focEnabled)
          .slot0Config(
              new Slot0Configs()
                  // feedforward
                  .withKS(5.0)
                  .withKV(0.0625)
                  .withKA(0.0)
                  // feedback
                  .withKP(5.0)
                  .withKI(0.0)
                  .withKD(0.0))
          .mmConfig(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(maximumShooterVelocity)
                  .withMotionMagicAcceleration(maximumShooterAcceleration))
          .currentLimitConfig(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(statorCurrentLimit)
                  .withStatorCurrentLimitEnable(false)
                  .withSupplyCurrentLimit(supplyCurrentLimit)
                  .withSupplyCurrentLowerLimit(supplyCurrentLimit)
                  .withSupplyCurrentLowerTime(0.1)
                  .withSupplyCurrentLimitEnable(false))
          .torqueCurrentConfigs(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(statorCurrentLimit)
                  .withPeakReverseTorqueCurrent(statorCurrentLimit.unaryMinus()))
          .build();

  public static final TalonFXConfigFollower flywheelFollowersConfig =
      TalonFXConfigFollower.builder()
          .leader(leaderShooterConfig)
          .followerCanId(22)
          .invertFollower(true)
          .build();

  public static final TalonFXConfigFollowerSim flywheelFollowersConfigSim =
      TalonFXConfigFollowerSim.builder()
          .hardware(flywheelFollowersConfig)
          .dcMotor(DCMotor.getKrakenX60Foc(2))
          .moi(0.002)
          .build();
}
