package org.robotzgarage.frc2026.subsystems.rollers.single;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.CurrentType;
import com.thethriftybot.devices.ThriftyNova.Error;
import com.thethriftybot.devices.ThriftyNova.PIDSlot;
import com.thethriftybot.util.Conversion;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.robotzgarage.frc2026.util.MotorConfigs.ThriftyNovaConfig;

public class SingleRollerIONova implements SingleRollerIO {
  private final ThriftyNova nova;
  private final Conversion positionConversion;
  private final Conversion velocityConversion;
  private final double reduction;

  private Angle positionGoal = Rotations.zero();

  public SingleRollerIONova(ThriftyNovaConfig config) {
    nova = new ThriftyNova(config.canId());
    nova.factoryReset();

    this.reduction = config.reduction();

    // Handles conversion from position math to encoder units
    positionConversion = new Conversion(config.positionUnit(), config.encoderType());
    velocityConversion = new Conversion(config.velocityUnit(), config.encoderType());

    // Handle overall config
    nova.setMaxCurrent(CurrentType.SUPPLY, config.currentLimitAmpsSupply())
        .setMaxCurrent(CurrentType.STATOR, config.currentLimitAmpsStator())
        .setRampUp(config.rampRateUp())
        .setRampDown(config.rampRateDown())
        .setMaxOutput(config.maxOutput())
        .setBrakeMode(config.isBrakeMode())
        .setInversion(config.invert())
        .enableSoftLimits(config.enableSoftLimits())
        .usePIDSlot(PIDSlot.SLOT0);

    // Handle PID control
    // nova.pid0.setP(config.pidConfig().p());
    // nova.pid0.setI(config.pidConfig().i());
    // nova.pid0.setD(config.pidConfig().d());
    // nova.pid0.setFF(config.pidConfig().ff());

    // NOTE Find better place to put errors
    for (Error error : nova.getErrors()) {
      System.err.println("Error" + error.toString());
    }

    nova.clearErrors();
  }

  @Override
  public void updateInputs(SingleRollerIOInputs inputs) {
    // We need a more clear way to check this
    inputs.connected = nova.getID() != 0;

    inputs.position = Rotations.of(positionConversion.fromMotor(nova.getPosition())).div(reduction);
    inputs.velocity =
        RotationsPerSecond.of(velocityConversion.fromMotor(nova.getVelocity())).div(reduction);

    inputs.appliedVoltage = nova.getVoltage();
    inputs.supplyCurrentAmps = nova.getSupplyCurrent();
    inputs.torqueCurrentAmps = nova.getStatorCurrent();
    inputs.temperatureCelsius = nova.getTemperature();

    inputs.positionGoal = positionGoal.div(reduction);
    inputs.positionSetpoint = Rotations.of(nova.getSetPoint()).div(reduction);
  }

  @Override
  public void runVolts(double volts) {
    this.nova.setVoltage(volts);
  }

  @Override
  public void runPercentOutput(double percent) {
    this.nova.setPercent(MathUtil.clamp(percent, -1.0, 1.0));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    this.nova.setVelocity(velocityConversion.toMotor(velocity.in(RotationsPerSecond)));
  }

  @Override
  public void setGoal(Angle position) {
    positionGoal = position.times(reduction);
    this.nova.setPosition(positionConversion.toMotor(positionGoal.in(Rotations)));
  }

  @Override
  public void resetPosition(Angle position) {
    this.nova.setEncoderPosition(positionConversion.toMotor(position.in(Rotations)));
  }

  @Override
  public void stop() {
    this.nova.stopMotor();
  }
}
