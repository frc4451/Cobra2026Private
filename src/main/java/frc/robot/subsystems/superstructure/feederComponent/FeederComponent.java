package frc.robot.subsystems.superstructure.feederComponent;

import org.robotzgarage.frc2026.subsystems.flywheel.Flywheel;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;

public class FeederComponent {
  private final String name = "/SuperStructure/FeederComponent";

  // Subsystems
  private final Flywheel feeder;
  private final Flywheel spiner;

  // Constructor to decide if it's real or sim
  public FeederComponent(SingleRollerIO feederIO, SingleRollerIO spinerIO) {
    feeder = new Flywheel(name + "/Feeder", feederIO, FeederConstants.flywheelConfig);
    spiner = new Flywheel(name + "/Spiner", spinerIO, SpinerConstants.flywheelConfig);
  }

  public void periodic() {
    // Periodic modes
    feeder.periodic();
    spiner.periodic();
  }

  // Component voids and booleans
  public void runPercentFeeder(double percent) {
    feeder.runPercentOutput(percent);
  }

  public void runPercentSpiner(double percent) {
    spiner.runPercentOutput(percent);
  }
}
