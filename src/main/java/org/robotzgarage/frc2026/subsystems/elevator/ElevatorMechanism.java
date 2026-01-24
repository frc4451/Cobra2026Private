package org.robotzgarage.frc2026.subsystems.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Builder;
import lombok.Getter;
import lombok.NonNull;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class ElevatorMechanism {
  @Builder
  public record ElevatorMechanismConfig(
      @NonNull String name, Angle initialAngle, double length, double width, Color8Bit color) {
    public ElevatorMechanismConfig {
      if (initialAngle == null) {
        initialAngle = Degrees.zero();
      }
      if (color == null) {
        color = new Color8Bit();
      }
    }
  }

  @Getter public final LoggedMechanismLigament2d elevatorMech;

  public ElevatorMechanism(ElevatorMechanismConfig config) {
    elevatorMech =
        new LoggedMechanismLigament2d(
            config.name(),
            config.length(),
            config.initialAngle().in(Degrees),
            config.width(),
            config.color());
  }

  public void update(Distance newHeight) {
    elevatorMech.setLength(newHeight.in(Meters));
  }
}
