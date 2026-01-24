package org.robotzgarage.frc2026.util;

import edu.wpi.first.math.geometry.Translation2d;
import lombok.Builder;

public class MechanismConfigs {
  @Builder
  public record RootMechanismConfig(
      double displayWidth, double displayHeight, Translation2d rootPosition) {
    public RootMechanismConfig {
      if (rootPosition == null) {
        rootPosition = new Translation2d(displayWidth / 2.0, 0.0);
      }
    }
  }
}
