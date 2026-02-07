package org.robotzgarage.frc2026.subsystems.rollers.follow;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;
import org.robotzgarage.frc2026.util.MotorConfigs.GenericPIDConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfigFollower;

public class FollowRollersIOTalonFX implements FollowRollersIO {
  protected final TalonFX leader;
  protected final TalonFX follower;

  protected final double reduction;
  protected Angle positionGoalRotations = Rotations.zero();

  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderVoltage;
  private final StatusSignal<Current> leaderSupplyCurrentAmps;
  private final StatusSignal<Current> leaderTorqueCurrentAmps;
  private final StatusSignal<Temperature> leaderTempCelsius;

  private final StatusSignal<Angle> followerPosition;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerVoltage;
  private final StatusSignal<Current> followerSupplyCurrentAmps;
  private final StatusSignal<Current> followerTorqueCurrentAmps;
  private final StatusSignal<Temperature> followerTempCelsius;

  private final StatusSignal<Double> positionSetpointRotations;
  private final StatusSignal<Double> velocitySetpointRotationsPerSec;

  private final MotionMagicVoltage mmVoltage;
  private final VoltageOut voltageOut;
  private final NeutralOut neutralOut = new NeutralOut();

  private final Follower followOut;

  private TalonFXConfiguration cfg;

  public FollowRollersIOTalonFX(TalonFXConfigFollower config) {
    this.reduction = config.leader().reduction();

    voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0).withEnableFOC(config.leader().foc());
    mmVoltage =
        new MotionMagicVoltage(0.0).withUpdateFreqHz(0).withEnableFOC(config.leader().foc());

    leader = new TalonFX(config.leader().canId(), config.leader().canBus());
    follower = new TalonFX(config.followerCanId(), config.leader().canBus());

    followOut =
        new Follower(
            config.leader().canId(),
            config.invertFollower() ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned);
    follower.setControl(followOut);

    leaderPosition = leader.getPosition();
    leaderVelocity = leader.getVelocity();
    leaderVoltage = leader.getMotorVoltage();
    leaderSupplyCurrentAmps = leader.getSupplyCurrent();
    leaderTorqueCurrentAmps = leader.getTorqueCurrent();
    leaderTempCelsius = leader.getDeviceTemp();

    positionSetpointRotations = leader.getClosedLoopReference();
    velocitySetpointRotationsPerSec = leader.getClosedLoopReferenceSlope();

    followerPosition = follower.getPosition();
    followerVelocity = follower.getVelocity();
    followerVoltage = follower.getMotorVoltage();
    followerSupplyCurrentAmps = follower.getSupplyCurrent();
    followerTorqueCurrentAmps = follower.getTorqueCurrent();
    followerTempCelsius = follower.getDeviceTemp();

    cfg = new TalonFXConfiguration();
    // spotless:off
    cfg.MotorOutput
        .withInverted(config.leader().invert() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(config.leader().isBrakeMode() ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    cfg.CurrentLimits = config.leader().currentLimitConfig();
    cfg.TorqueCurrent = config.leader().torqueCurrentConfigs();
    cfg.Slot0 = config.leader().slot0Config();
    cfg.MotionMagic = config.leader().mmConfig();
    cfg.SoftwareLimitSwitch = config.leader().softwareLimitSwitch();
    // spotless:on

    BaseStatusSignal.setUpdateFrequencyForAll(
        config.leader().phoenixUpdateFreqHz(),
        leaderPosition,
        leaderVelocity,
        leaderVoltage,
        leaderSupplyCurrentAmps,
        leaderTorqueCurrentAmps,
        leaderTempCelsius,
        followerPosition,
        followerVelocity,
        followerVoltage,
        followerSupplyCurrentAmps,
        followerTorqueCurrentAmps,
        followerTempCelsius,
        positionSetpointRotations,
        velocitySetpointRotationsPerSec);
    leader.optimizeBusUtilization(0.0, 1.0);
    follower.optimizeBusUtilization(0.0, 1.0);

    leader.getConfigurator().apply(cfg);
    follower.getConfigurator().apply(cfg);
  }

  @Override
  public void updateInputs(FollowRollersMagicIOInputs inputs) {
    inputs.leaderConnected =
        BaseStatusSignal.refreshAll(
                leaderPosition,
                leaderVelocity,
                leaderVoltage,
                leaderSupplyCurrentAmps,
                leaderTorqueCurrentAmps,
                leaderTempCelsius,
                positionSetpointRotations,
                velocitySetpointRotationsPerSec)
            .isOK();

    inputs.followerConnected =
        BaseStatusSignal.refreshAll(
                followerPosition,
                followerVelocity,
                followerVoltage,
                followerSupplyCurrentAmps,
                followerTorqueCurrentAmps,
                followerTempCelsius)
            .isOK();

    inputs.leaderPosition = leaderPosition.getValue().div(reduction);
    inputs.leaderVelocity = leaderVelocity.getValue().div(reduction);

    inputs.leaderAppliedVoltage = leaderVoltage.getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leaderSupplyCurrentAmps.getValueAsDouble();
    inputs.leaderTorqueCurrentAmps = leaderTorqueCurrentAmps.getValueAsDouble();
    inputs.leaderTemperatureCelsius = leaderTempCelsius.getValueAsDouble();

    inputs.followerPosition = followerPosition.getValue().div(reduction);
    inputs.followerVelocity = followerVelocity.getValue().div(reduction);

    inputs.followerAppliedVoltage = followerVoltage.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupplyCurrentAmps.getValueAsDouble();
    inputs.followerTorqueCurrentAmps = followerTorqueCurrentAmps.getValueAsDouble();
    inputs.followerTemperatureCelsius = followerTempCelsius.getValueAsDouble();

    inputs.positionGoal = positionGoalRotations.div(reduction);
    inputs.positionSetpoint =
        Rotations.of(positionSetpointRotations.getValueAsDouble()).div(reduction);
    inputs.velocitySetpoint =
        RotationsPerSecond.of(velocitySetpointRotationsPerSec.getValueAsDouble()).div(reduction);
  }

  @Override
  public void runVolts(double volts) {
    leader.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setGoal(Angle position) {
    positionGoalRotations = position.times(reduction);
    leader.setControl(mmVoltage.withPosition(positionGoalRotations));
  }

  @Override
  public void resetPosition(Angle position) {
    leader.setPosition(position.times(reduction));
    follower.setPosition(position.times(reduction));
  }

  @Override
  public void stop() {
    leader.setControl(neutralOut);
  }

  @Override
  public GenericPIDConfig readPID() {
    return new GenericPIDConfig(cfg.Slot0.kP, cfg.Slot0.kI, cfg.Slot0.kD, 0.0);
  }

  @Override
  public void updatePID(double kP, double kI, double kD) {
    cfg.Slot0.kP = kP;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = kD;

    PhoenixUtil.tryUntilOk(5, () -> leader.getConfigurator().apply(cfg));
    PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(cfg));
  }
}
