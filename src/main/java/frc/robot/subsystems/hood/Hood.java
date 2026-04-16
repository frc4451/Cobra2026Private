package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.bobot_state.BobotState;
import org.littletonrobotics.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.subsystems.pivot.Pivot;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;

public class Hood extends SubsystemBase {
  private static final String name = "/Hood";

  public enum HoodState {
    DOWN,
    MID,
    UP
  }

  private static final LoggedTunableNumber angleDown =
      new LoggedTunableNumber(name + "/Angle/Down", 0.0);
  private static final LoggedTunableNumber angleMid =
      new LoggedTunableNumber(name + "/Angle/Mid", 1.0);
  private static final LoggedTunableNumber angleUp =
      new LoggedTunableNumber(name + "/Angle/Up", 2.0);

  private final Pivot hood;
  private HoodState currentState = HoodState.DOWN;

  public Hood(SingleRollerIO hoodIO) {
    hood = new Pivot(name, hoodIO, HoodConstants.pivotConfig);
  }

  @Override
  public void periodic() {
    hood.periodic();
    Logger.recordOutput(name + "/State", currentState.name());
  }

  public Command setHoodState(HoodState state) {
    return runOnce(
        () -> {
          currentState = state;
          double angle =
              switch (state) {
                case DOWN -> angleDown.get();
                case MID -> angleMid.get();
                case UP -> angleUp.get();
              };
          hood.setGoal(Degrees.of(angle));
        });
  }

  public Trigger isNearGoal(Angle tolerance) {
    return new Trigger(() -> this.hood.isNear(this.hood.getGoalPosition(), tolerance));
  }

  public Trigger isNearTrench() {
    return BobotState.isNearTrench();
  }
}
