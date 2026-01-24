package org.robotzgarage.frc2026.subsystems.pivot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Builder;
import lombok.Getter;
import lombok.NonNull;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class PivotMechanism {
  @Builder
  public record PivotMechanismConfig(
      @NonNull String name, Angle initialAngle, double length, double width, Color8Bit color) {
    public PivotMechanismConfig {
      if (initialAngle == null) {
        initialAngle = Degrees.zero();
      }
      if (color == null) {
        color = new Color8Bit();
      }
    }
  }

  @Getter public final LoggedMechanismLigament2d pivotMech;

  public PivotMechanism(PivotMechanismConfig config) {
    pivotMech =
        new LoggedMechanismLigament2d(
            config.name(),
            config.length(),
            config.initialAngle().in(Degrees),
            config.width(),
            config.color());
  }

  public void update(Angle newAngle) {
    pivotMech.setAngle(newAngle.in(Degrees));
  }
}
