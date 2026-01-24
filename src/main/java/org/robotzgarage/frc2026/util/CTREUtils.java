package org.robotzgarage.frc2026.util;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class CTREUtils {
  public static class MotionMagicHelpers {
    /**
     * Idiomatic wrapper for Angular Position for `Rotations`
     *
     * @return rotations
     */
    public static double getRotations(double magnitude) {
      return Rotations.of(magnitude).in(Rotations);
    }

    /**
     * Idiomatic wrapper for Angular Velocity for `Rotations`
     *
     * @return rotations / second
     */
    public static double getRotationsVelocity(double magnitude) {
      return RotationsPerSecond.of(magnitude).in(RotationsPerSecond);
    }

    /**
     * Idiomatic wrapper for Angular Acceleration for `Rotations`
     *
     * @return rotations / (second^2)
     */
    public static double getRotationsAcceleration(double magnitude) {
      return RotationsPerSecondPerSecond.of(magnitude).in(RotationsPerSecondPerSecond);
    }

    /**
     * Idiomatic wrapper for Angular Jerk for `Rotations`
     *
     * @return rotations / (second^3)
     */
    public static double getRotationsJerk(double magnitude) {
      return RotationsPerSecondPerSecond.per(Second)
          .of(magnitude)
          .in(RotationsPerSecondPerSecond.per(Second));
    }

    public static double getVoltage(double magnitude) {
      return Volts.of(magnitude).in(Volts);
    }
  }
}
