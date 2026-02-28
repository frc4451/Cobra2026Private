package frc.robot.bobot_state.dashboard;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.util.VirtualSubsystem;
import org.spectrum3847.ShiftHelpers;

public class TeleopShiftManager extends VirtualSubsystem {
  public enum TeleopShift {
    TRANSITION,
    RED,
    BLUE,
    ENDGAME
  }

  private TeleopShift currentShift = TeleopShift.TRANSITION;

  @Override
  public void periodic() {
    Logger.recordOutput("TeleopShift/CurrentShift", isYourShift());
    Logger.recordOutput("TeleopShift/TimeLeft", getTimeLeft());
  }


  public boolean isYourShift() {
    return ShiftHelpers.currentShiftIsYours();
  }

  public int getTimeLeft() {
    return ShiftHelpers.timeLeftInShiftSeconds(DriverStation.getMatchTime());
  }

  @Override
  public void simulationPeriodic() {}
}
