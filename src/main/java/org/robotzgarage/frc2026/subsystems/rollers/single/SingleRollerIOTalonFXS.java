package org.robotzgarage.frc2026.subsystems.rollers.single;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;
import org.robotzgarage.frc2026.util.MotorConfigs.GenericFeedForwardConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.GenericFeedbackConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXSConfig;

public class SingleRollerIOTalonFXS implements SingleRollerIO {
  protected final TalonFXS talon;
  protected final double reduction;

  protected final StatusSignal<Angle> position;
  protected final StatusSignal<AngularVelocity> velocity;
  protected final StatusSignal<Voltage> voltage;
  protected final StatusSignal<Current> supplyCurrentAmps;
  protected final StatusSignal<Current> torqueCurrentAmps;
  protected final StatusSignal<Temperature> tempCelsius;

  protected final StatusSignal<Double> positionSetpointRotations;
  protected final StatusSignal<Double> velocitySetpointRotationsPerSec;

  protected final MotionMagicVoltage mmVoltage;
  protected final MotionMagicVelocityVoltage mmVelocityVoltage;

  protected final VoltageOut voltageOut;
  protected final DutyCycleOut dutyCycleOut;
  protected final NeutralOut neutralOut = new NeutralOut();

  protected Angle positionGoal = Rotations.zero();
  protected TalonFXSConfiguration cfg;

  public SingleRollerIOTalonFXS(TalonFXSConfig config) {
    this.reduction = config.reduction();

    talon = new TalonFXS(config.canId(), config.canBus());

    voltageOut = new VoltageOut(0.0).withEnableFOC(config.foc());
    dutyCycleOut = new DutyCycleOut(0).withEnableFOC(config.foc());
    mmVoltage = new MotionMagicVoltage(0.0).withEnableFOC(config.foc());
    mmVelocityVoltage = new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(config.foc());

    position = talon.getPosition();
    velocity = talon.getVelocity();
    voltage = talon.getMotorVoltage();
    supplyCurrentAmps = talon.getSupplyCurrent();
    torqueCurrentAmps = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    positionSetpointRotations = talon.getClosedLoopReference();
    velocitySetpointRotationsPerSec = talon.getClosedLoopReferenceSlope();

    cfg = new TalonFXSConfiguration();
    // spotless:off
    cfg.MotorOutput
        .withInverted(config.invert() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(config.isBrakeMode() ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    cfg.CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(config.currentLimitAmps());
    cfg.Slot0 = config.slot0Config();
    cfg.MotionMagic = config.mmConfig();
    cfg.SoftwareLimitSwitch = config.softwareLimitSwitch();
    cfg.Commutation.MotorArrangement = config.motorJST();
    cfg.ExternalFeedback = new ExternalFeedbackConfigs().withSensorToMechanismRatio(reduction);
    // spotless:on

    BaseStatusSignal.setUpdateFrequencyForAll(
        config.phoenixUpdateFreqHz(),
        position,
        velocity,
        voltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        tempCelsius,
        positionSetpointRotations,
        velocitySetpointRotationsPerSec);
    talon.optimizeBusUtilization(0.0, 1.0);

    talon.getConfigurator().apply(cfg);
  }

  @Override
  public void updateInputs(SingleRollerIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position,
                velocity,
                voltage,
                supplyCurrentAmps,
                torqueCurrentAmps,
                tempCelsius,
                positionSetpointRotations,
                velocitySetpointRotationsPerSec)
            .isOK();

    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue();

    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = tempCelsius.getValueAsDouble();

    inputs.positionGoal = positionGoal.div(reduction);
    inputs.positionSetpoint = Rotations.of(positionSetpointRotations.getValue());
    inputs.velocitySetpoint =
        RotationsPerSecond.of(velocitySetpointRotationsPerSec.getValueAsDouble());
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runPercentOutput(double percent) {
    talon.setControl(dutyCycleOut.withOutput(percent));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    talon.setControl(mmVelocityVoltage.withVelocity(velocity));
  }

  @Override
  public void setGoal(Angle position) {
    positionGoal = position;
    talon.setControl(mmVoltage.withPosition(positionGoal));
  }

  @Override
  public void resetPosition(Angle position) {
    talon.setPosition(position.times(reduction));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public GenericFeedbackConfig readFeedBack() {
    return new GenericFeedbackConfig(cfg.Slot0.kP, cfg.Slot0.kI, cfg.Slot0.kD);
  }

  @Override
  public void updateFeedBack(GenericFeedbackConfig fbConfig) {
    cfg.Slot0.kP = fbConfig.p();
    cfg.Slot0.kI = fbConfig.i();
    cfg.Slot0.kD = fbConfig.d();

    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(cfg));
  }

  @Override
  public GenericFeedForwardConfig readFeedForward() {
    return new GenericFeedForwardConfig(cfg.Slot0.kS, cfg.Slot0.kG, cfg.Slot0.kV, cfg.Slot0.kA);
  }

  @Override
  public void updateFeedForward(GenericFeedForwardConfig ffConfig) {
    cfg.Slot0.kS = ffConfig.s();
    cfg.Slot0.kG = ffConfig.g();
    cfg.Slot0.kV = ffConfig.v();
    cfg.Slot0.kA = ffConfig.a();

    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(cfg));
  }
}
