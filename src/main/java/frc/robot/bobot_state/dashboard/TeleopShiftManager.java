package frc.robot.bobot_state.dashboard;

import org.robotzgarage.frc2026.util.FieldUtils;
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
        
    }

    public boolean isYourShift() {
        return ShiftHelpers.currentShiftIsYours();
    }

    @Override
    public void simulationPeriodic() {}
}
