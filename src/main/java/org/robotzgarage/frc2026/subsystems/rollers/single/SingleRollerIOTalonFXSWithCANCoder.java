package org.robotzgarage.frc2026.subsystems.rollers.single;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;
import org.robotzgarage.frc2026.util.MotorConfigs.CANCoderConfig;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXSConfig;

public class SingleRollerIOTalonFXSWithCANCoder implements SingleRollerIO {
  protected final TalonFXS talon;
  protected final CANcoder cancoder;
  protected final double totalReduction;
  protected final double sensorToMechanismReduction;
  protected final double rotorToSensorReduction;

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

  public SingleRollerIOTalonFXSWithCANCoder(TalonFXSConfig config, CANCoderConfig canCoderConfig) {
    this.rotorToSensorReduction = canCoderConfig.rotorToSensorReduction();
    this.sensorToMechanismReduction = canCoderConfig.sensorToMechanismReduction();
    this.totalReduction =
        canCoderConfig.rotorToSensorReduction() * canCoderConfig.sensorToMechanismReduction();

    talon = new TalonFXS(config.canId(), config.canBus());
    cancoder = new CANcoder(canCoderConfig.canId(), canCoderConfig.canBus());

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

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset =
        canCoderConfig.magnetOffset().in(Rotations) * canCoderConfig.sensorToMechanismReduction();
    encoderConfig.MagnetSensor.SensorDirection =
        canCoderConfig.isClockwisePositive()
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;

    if (canCoderConfig.discontinuity() != null) {
      encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = canCoderConfig.discontinuity();
    }

    // encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
    //     MathUtil.inputModulus(
    //         canCoderConfig.maxAngle().in(Rotations) + canCoderConfig.minAngle().in(Rotations), 0,
    // 1)
    // / 2.0

    PhoenixUtil.tryUntilOk(5, () -> cancoder.getConfigurator().apply(encoderConfig, 0.25));

    TalonFXSConfiguration cfg = new TalonFXSConfiguration();
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
    cfg.ExternalFeedback = new ExternalFeedbackConfigs()
      .withFusedCANcoder(cancoder)
      .withRotorToSensorRatio(canCoderConfig.rotorToSensorReduction())
      .withSensorToMechanismRatio(canCoderConfig.sensorToMechanismReduction());
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

    inputs.positionGoal = positionGoal;
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
    talon.setPosition(position);
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }
}
