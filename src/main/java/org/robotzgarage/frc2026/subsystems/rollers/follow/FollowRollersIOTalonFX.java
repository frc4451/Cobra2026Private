package org.robotzgarage.frc2026.subsystems.rollers.follow;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfigFollower;

public class FollowRollersIOTalonFX implements FollowRollersIO {
  protected final TalonFX leader;
  protected final TalonFX follower;

  protected final double reduction;
  protected Angle positionGoalRotations = Rotations.zero();

  protected final StatusSignal<Angle> leaderPosition;
  protected final StatusSignal<AngularVelocity> leaderVelocity;
  protected final StatusSignal<Voltage> leaderVoltage;
  protected final StatusSignal<Current> leaderSupplyCurrentAmps;
  protected final StatusSignal<Current> leaderTorqueCurrentAmps;
  protected final StatusSignal<Temperature> leaderTempCelsius;

  protected final StatusSignal<Angle> followerPosition;
  protected final StatusSignal<AngularVelocity> followerVelocity;
  protected final StatusSignal<Voltage> followerVoltage;
  protected final StatusSignal<Current> followerSupplyCurrentAmps;
  protected final StatusSignal<Current> followerTorqueCurrentAmps;
  protected final StatusSignal<Temperature> followerTempCelsius;

  protected final StatusSignal<Double> positionSetpointRotations;
  protected final StatusSignal<Double> velocitySetpointRotationsPerSec;

  protected final MotionMagicVoltage mmVoltage;
  protected final VoltageOut voltageOut;
  protected final NeutralOut neutralOut = new NeutralOut();
  protected final TorqueCurrentFOC torqueCurrentFOC;
  protected final DutyCycleOut dutyCycleOut;

  protected final Follower followOut;

  public FollowRollersIOTalonFX(TalonFXConfigFollower config) {
    this.reduction = config.leader().reduction();

    voltageOut = new VoltageOut(0.0).withEnableFOC(config.leader().foc());
    mmVoltage = new MotionMagicVoltage(0.0).withEnableFOC(config.leader().foc());
    dutyCycleOut = new DutyCycleOut(0.0).withEnableFOC(config.leader().foc());
    torqueCurrentFOC = new TorqueCurrentFOC(Amps.of(0.0));

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

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    // spotless:off
    cfg.MotorOutput
        .withInverted(config.leader().invert() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(config.leader().isBrakeMode() ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    cfg.CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(config.leader().currentLimitAmps());
    cfg.Slot0 = config.leader().slot0Config();
    cfg.MotionMagic = config.leader().mmConfig();
    cfg.SoftwareLimitSwitch = config.leader().softwareLimitSwitch();
    cfg.Feedback = new FeedbackConfigs()
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
      .withSensorToMechanismRatio(reduction);
    // spotless:on

    PhoenixUtil.tryUntilOk(
        5,
        () ->
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
                velocitySetpointRotationsPerSec));

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

    inputs.leaderPosition = leaderPosition.getValue();
    inputs.leaderVelocity = leaderVelocity.getValue();

    inputs.leaderAppliedVoltage = leaderVoltage.getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leaderSupplyCurrentAmps.getValueAsDouble();
    inputs.leaderTorqueCurrentAmps = leaderTorqueCurrentAmps.getValueAsDouble();
    inputs.leaderTemperatureCelsius = leaderTempCelsius.getValueAsDouble();

    inputs.followerPosition = followerPosition.getValue();
    inputs.followerVelocity = followerVelocity.getValue();

    inputs.followerAppliedVoltage = followerVoltage.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupplyCurrentAmps.getValueAsDouble();
    inputs.followerTorqueCurrentAmps = followerTorqueCurrentAmps.getValueAsDouble();
    inputs.followerTemperatureCelsius = followerTempCelsius.getValueAsDouble();

    inputs.positionGoal = positionGoalRotations;
    inputs.positionSetpoint = Rotations.of(positionSetpointRotations.getValueAsDouble());
    inputs.velocitySetpoint =
        RotationsPerSecond.of(velocitySetpointRotationsPerSec.getValueAsDouble());
  }

  @Override
  public void runVolts(double volts) {
    leader.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runPercentOutput(double percent) {
    leader.setControl(dutyCycleOut.withOutput(percent));
  }

  @Override
  public void runTorqueCurrent(Current current) {
    leader.setControl(torqueCurrentFOC.withOutput(current));
  }

  @Override
  public void setGoal(Angle position) {
    positionGoalRotations = position;
    leader.setControl(mmVoltage.withPosition(positionGoalRotations));
  }

  @Override
  public void resetPosition(Angle position) {
    leader.setPosition(position);
    follower.setPosition(position);
  }

  @Override
  public void stop() {
    leader.setControl(neutralOut);
  }
}
