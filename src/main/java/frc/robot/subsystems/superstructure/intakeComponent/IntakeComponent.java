package frc.robot.subsystems.superstructure.intakeComponent;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import org.robotzgarage.frc2026.subsystems.flywheel.Flywheel;
import org.robotzgarage.frc2026.subsystems.pivot.Pivot;
import org.robotzgarage.frc2026.subsystems.pivot.PivotMechanism;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;

public class IntakeComponent {
  private final String name = "/SuperStructure/IntakeComponent";

  // Subsystems
  private final Pivot endeffector;
  private final Flywheel intake;

  // Mechanisms
  private final PivotingMechanism measuredMechanism;
  private final PivotingMechanism setpointMechanism;
  private final PivotingMechanism goalMechanism;

  // Constructor to decide if it's real or sim
  public IntakeComponent(SingleRollerIO endeffectorIO, SingleRollerIO intakeIO) {
    endeffector = new Pivot(name + "/Endeffector", endeffectorIO, EndeffectorConstants.pivotConfig);
    intake = new Flywheel(name + "/Intake", intakeIO, IntakeConstants.flywheelConfig);

    // Connecting mechanisms for logging and organization
    {
      String name = endeffector.getName() + "/Mechanisms";
      this.goalMechanism =
          new PivotingMechanism(
              name + "/GoalMechanism",
              new PivotMechanism(EndeffectorConstants.goalPivotMechanismConfig));

      this.measuredMechanism =
          new PivotingMechanism(
              name + "/MeasuredMechanism",
              new PivotMechanism(EndeffectorConstants.measuredPivotMechanismConfig));

      this.setpointMechanism =
          new PivotingMechanism(
              name + "/SetpointMechanism",
              new PivotMechanism(EndeffectorConstants.setpointPivotMechanismConfig));
    }
  }

  public void periodic() {
    // Periodic modes
    endeffector.periodic();
    intake.periodic();

    // Updating mechanisms
    goalMechanism.update(endeffector.getGoalPosition());
    measuredMechanism.update(endeffector.getPosition());
    setpointMechanism.update(endeffector.getPositionSetpoint());
  }

  public void setEndeffectorAngle(Angle angle) {
    endeffector.setGoalPosition(angle);
  }

  public void runIntakePercent(double percent) {
    intake.runPercentOutput(percent);
  }

  public boolean isEndeffectorOut() {
    return endeffector.isNear(Degrees.of(90));
  }

  public boolean isEndeffectorIn() {
    return endeffector.isNear(Degrees.of(0));
  }
}
