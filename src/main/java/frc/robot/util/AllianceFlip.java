package frc.robot.util;

import org.robotzgarage.frc2026.util.FieldUtils;

public record AllianceFlip<T>(T blue, T red) {
  public T getValue() {
    return FieldUtils.isBlueAlliance() ? blue : red;
  }
}
