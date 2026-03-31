package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.LoggedTunableNumber;
import org.robotzgarage.frc2026.subsystems.pivot.Pivot;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRoller;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;
import org.robotzgarage.frc2026.util.TuningUtils.TuningUnit;

public class Hood extends SubsystemBase {
  private static final String name = "/Hood";

  private static final LoggedTunableNumber hoodSetAngle =
      new LoggedTunableNumber(name + "/HoodSetAngle", 28.0);

  private final Pivot hood;

  public Hood(SingleRollerIO hoodIO) {
    hood = new Pivot(name + "/Hoodsfgh", hoodIO, HoodConstants.pivotConfig);
  }

  public void period() {
    hood.periodic();
  }

  public Command setHoodAngle() {
    return runOnce(() -> hood.setGoal(Degrees.of(hoodSetAngle.get())));
  }

  public Trigger isNearGoal(Angle tolerance) {
    return new Trigger(() -> this.hood.isNear(this.hood.getGoalPosition(), tolerance));
  }

  public Command stopPivot() {
    return runOnce(() -> hood.stop());
  }

  public Command endeffectorFeedForwardCharacterization(TuningUnit unit) {
    return this.hood.quasistaticCharacterization(unit);
  }
}
