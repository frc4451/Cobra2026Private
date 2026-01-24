package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.feederComponent.FeederComponent;
import frc.robot.subsystems.superstructure.intakeComponent.IntakeComponent;
import frc.robot.subsystems.superstructure.shooterComponent.ShooterComponent;

public class SuperStructure extends SubsystemBase {
  private final String name = "/SuperStructure";

  // Components
  private final ShooterComponent shooterComponent;
  private final FeederComponent feederComponent;
  private final IntakeComponent intakeComponent;

  // Constructor to decide if it's real or sim
  public SuperStructure(
      ShooterComponent shooterComponent,
      FeederComponent feederComponent,
      IntakeComponent intakeComponent) {
    this.shooterComponent = shooterComponent;
    this.feederComponent = feederComponent;
    this.intakeComponent = intakeComponent;
  }

  @Override
  public void periodic() {
    this.shooterComponent.periodic();
    this.feederComponent.periodic();
    this.intakeComponent.periodic();
  }

  // Shooter commands
  public Command zeroShooter() {
    return Commands.runOnce(() -> shooterComponent.zeroHoodnTurret());
  }

  public Command setShooter(double hood, double turret) {
    return Commands.runOnce(() -> shooterComponent.setHoodnTurret(hood, turret));
  }

  public Command runShooterCommand() {
    return Commands.runEnd(
        () -> shooterComponent.runShooterPercent(0.5), () -> shooterComponent.runShooterPercent(0));
  }

  public Command runShooterCommand(double var) {
    return Commands.runEnd(
        () -> shooterComponent.runShooterPercent(var), () -> shooterComponent.runShooterPercent(0));
  }

  public Trigger isShooterReady() {
    return new Trigger(() -> shooterComponent.isShooterReady());
  }

  // Intake commands
  public Command endeffectorOut() {
    return Commands.runOnce(() -> intakeComponent.setEndeffectorAngle(Degrees.of(90)));
  }

  public Command endeffectorIn() {
    return Commands.runOnce(() -> intakeComponent.setEndeffectorAngle(Degrees.of(0)));
  }

  public Command runIntakeCommand() {
    return Commands.runEnd(
        () -> intakeComponent.runIntakePercent(0.5), () -> intakeComponent.runIntakePercent(0.0));
  }

  public Trigger isEndeffectorOut() {
    return new Trigger(() -> intakeComponent.isEndeffectorOut());
  }

  public Trigger isEndeffectorIn() {
    return new Trigger(() -> intakeComponent.isEndeffectorIn());
  }

  // Feeder commands
  public Command runFeederCommand() {
    return Commands.runEnd(
        () -> feederComponent.runPercentFeeder(0.5), () -> feederComponent.runPercentFeeder(0.0));
  }

  public Command runSpinerCommand() {
    return Commands.runEnd(
        () -> feederComponent.runPercentSpiner(0.5), () -> feederComponent.runPercentSpiner(0.0));
  }

  public Command runFeederCommand(double var) {
    return Commands.runEnd(
        () -> feederComponent.runPercentFeeder(var), () -> feederComponent.runPercentFeeder(0.0));
  }

  public Command runSpinerCommand(double var) {
    return Commands.runEnd(
        () -> feederComponent.runPercentSpiner(var), () -> feederComponent.runPercentSpiner(0.0));
  }
}
