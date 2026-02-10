package frc.robot.bobot_state.varc;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.AllianceFlip;

public class VARCConstants {

  // The offset needed for going over the bump, so we don't tip with the intake
  public static final AllianceFlip<Rotation2d> leftBumpRotation =
      new AllianceFlip<Rotation2d>(Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(45));
  public static final AllianceFlip<Rotation2d> rightBumpRotation =
      new AllianceFlip<Rotation2d>(Rotation2d.fromDegrees(-135), Rotation2d.fromDegrees(-45));
}
