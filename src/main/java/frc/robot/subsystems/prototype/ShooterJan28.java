package frc.robot.subsystems.prototype;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.robotzgarage.frc2026.subsystems.flywheel.Flywheel;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;

public class ShooterJan28 extends SubsystemBase {
  private final String name = "/ShooterJan28";

  // Subsystems
  private final Flywheel leader;
  private final Flywheel follower;
  private final Flywheel feeder;

  // Constructor to decide if it's real or sim
  public ShooterJan28(SingleRollerIO leaderIO, SingleRollerIO followerIO, SingleRollerIO feederIO) {
    leader = new Flywheel(name + "/leader", leaderIO, ShooterJan28Constants.shooterFlywheelConfig);
    follower =
        new Flywheel(name + "/follower", followerIO, ShooterJan28Constants.shooterFlywheelConfig);
    feeder = new Flywheel(name + "/feeder", feederIO, ShooterJan28Constants.feederFlywheelConfig);
  }

  public void periodic() {
    // Periodic modes
    leader.periodic();
    follower.periodic();
    feeder.periodic();
  }

  //   // Component voids and booleans
  //   public void runPercentFeeder(double percent) {
  //     feeder.runPercentOutput(percent);
  //   }

  //   public void runPercentSpiner(double percent) {
  //     spiner.runPercentOutput(percent);
  //   }

  public Command runShooterFlywheels(double percent) {
    return run(
        () -> {
          leader.runPercentOutput(percent);
          follower.runPercentOutput(percent);
        });
  }

  public Command runALLL(double percent) {
    return run(
        () -> {
          leader.runPercentOutput(percent);
          follower.runPercentOutput(percent);
          feeder.runPercentOutput(percent);
        });
  }

    public Command runALLL(double shooterPercent, double feederPercent) {
    return run(
        () -> {
          leader.runPercentOutput(shooterPercent);
          follower.runPercentOutput(shooterPercent);
          feeder.runPercentOutput(feederPercent);
        });
  }

    public Command runFeeder(double percent) {
    return run(
        () -> feeder.runPercentOutput(percent));
  }

  public Command stop() {
    return run(
        () -> {
          leader.stop();
          follower.stop();
          feeder.stop();
        });
  }
}
