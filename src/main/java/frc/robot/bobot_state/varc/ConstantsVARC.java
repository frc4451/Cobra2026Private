package frc.robot.bobot_state.varc;

import edu.wpi.first.math.geometry.Rotation2d;

public class ConstantsVARC {

  // The offset needed for going over the bump, so we don't tip with the intake
  public static final Rotation2d leftBumpRotation = Rotation2d.fromDegrees(135);
  public static final Rotation2d rightBumpRotation = Rotation2d.fromDegrees(-135);
}
