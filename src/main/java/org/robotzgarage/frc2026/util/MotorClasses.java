package org.robotzgarage.frc2026.util;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * These are for motors that are not yet supported by WPILib, but we have on-hand
 *
 * <p>Minion and Kraken X44 are supplied by YAMGen, Pulsar 775 is pulled from Spec Sheet
 */
public class MotorClasses {
  /** WCP Minion, controlled with Talon FXS */
  public static final DCMotor minion =
      new DCMotor(12, 3.1, 200.46, 1.43, Units.rotationsPerMinuteToRadiansPerSecond(7200), 1);

  /** Kraken X44, TalonFX */
  public static final DCMotor krakenX44 =
      new DCMotor(12, 4.05, 275, 1.4, Units.rotationsPerMinuteToRadiansPerSecond(7530), 1);

  /** Thrifty Pulsar 775, controlled with Thrifty Nova Controller */
  public static final DCMotor pulsar775 =
      new DCMotor(12, 3.1, 189, 2.7, Units.rotationsPerMinuteToRadiansPerSecond(7500), 1);
}
