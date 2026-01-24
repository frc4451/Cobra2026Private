package frc.robot.subsystems.superstructure.shooterComponent;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.subsystems.superstructure.shooterComponent.shooterComponentConstan.HoodConstants;
import frc.robot.subsystems.superstructure.shooterComponent.shooterComponentConstan.ShooterConstants;
import frc.robot.subsystems.superstructure.shooterComponent.shooterComponentConstan.TurretConstants;
import org.robotzgarage.frc2026.subsystems.flywheel.Flywheel;
import org.robotzgarage.frc2026.subsystems.pivot.Pivot;
import org.robotzgarage.frc2026.subsystems.pivot.PivotMechanism;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;

public class ShooterComponent {
  private final String name = "/SuperStructure/ShooterComponent";

  // Subsystems
  private final Pivot hood;
  private final Pivot turret;
  private final Flywheel shooter;

  // Mechanisms
  private final PivotingMechanism measuredHoodMechanism;
  private final PivotingMechanism setpointHoodMechanism;
  private final PivotingMechanism goalHoodMechanism;

  private final PivotingMechanism measuredTurretMechanism;
  private final PivotingMechanism setpointTurretMechanism;
  private final PivotingMechanism goalTurretMechanism;

  // Constructor to decide if it's real or sim
  public ShooterComponent(
      SingleRollerIO hoodIO, SingleRollerIO turretIO, SingleRollerIO shooterIO) {

    hood = new Pivot(name + "/Hood", hoodIO, HoodConstants.pivotConfig);
    turret = new Pivot(name + "/Turret", turretIO, TurretConstants.pivotConfig);
    shooter = new Flywheel(name + "/Shooter", shooterIO, ShooterConstants.flywheelConfig);

    // Connecting mechanisms for logging and organization
    {
      String name = hood.getName() + "/Mechanisms";
      this.goalHoodMechanism =
          new PivotingMechanism(
              name + "/GoalMechanism", new PivotMechanism(HoodConstants.goalPivotMechanismConfig));

      this.measuredHoodMechanism =
          new PivotingMechanism(
              name + "/MeasuredMechanism",
              new PivotMechanism(HoodConstants.measuredPivotMechanismConfig));

      this.setpointHoodMechanism =
          new PivotingMechanism(
              name + "/SetpointMechanism",
              new PivotMechanism(HoodConstants.setpointPivotMechanismConfig));
    }

    {
      String name = turret.getName() + "/Mechanisms";
      this.goalTurretMechanism =
          new PivotingMechanism(
              name + "/GoalMechanism",
              new PivotMechanism(TurretConstants.goalPivotMechanismConfig));

      this.measuredTurretMechanism =
          new PivotingMechanism(
              name + "/MeasuredMechanism",
              new PivotMechanism(TurretConstants.measuredPivotMechanismConfig));

      this.setpointTurretMechanism =
          new PivotingMechanism(
              name + "/SetpointMechanism",
              new PivotMechanism(TurretConstants.setpointPivotMechanismConfig));
    }
  }

  public void periodic() {
    // Periodic modes
    hood.periodic();
    turret.periodic();
    shooter.periodic();

    // Updating mechanisms
    goalHoodMechanism.update(hood.getGoalPosition());
    measuredHoodMechanism.update(hood.getPosition());
    setpointHoodMechanism.update(hood.getPositionSetpoint());

    goalTurretMechanism.update(turret.getGoalPosition());
    measuredTurretMechanism.update(turret.getPosition());
    setpointTurretMechanism.update(turret.getPositionSetpoint());
  }

  public void zeroHoodnTurret() {
    hood.setGoalPosition(Degrees.of(0.0));
    turret.setGoalPosition(Degrees.of(0.0));
  }

  public void setHoodAngle(double angle) {
    hood.setGoalPosition(Degrees.of(angle));
  }

  public void setTurretAngle(double angle) {
    hood.setGoalPosition(Degrees.of(angle));
  }

  public void setHoodnTurret(double hoo, double tur) {
    hood.setGoalPosition(Degrees.of(hoo));
    turret.setGoalPosition(Degrees.of(tur));
  }

  public void runShooterPercent(double percent) {
    shooter.setPercentOutput(percent);
  }

  public boolean isShooterReady() {
    return shooter.isNear(RotationsPerSecond.of(10.0));
  }
}
