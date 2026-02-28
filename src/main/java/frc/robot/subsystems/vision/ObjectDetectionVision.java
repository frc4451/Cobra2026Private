package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.Alert;
import frc.robot.Constants;
import org.robotzgarage.frc2026.subsystems.vision.VisionTypes.VisionSource;
import org.robotzgarage.frc2026.util.VirtualSubsystem;

public class ObjectDetectionVision extends VirtualSubsystem {
  public static record ObjectDetectionCamera(
      ObjectDetectionIO io,
      ObjectDetectionIOInputsAutoLogged inputs,
      VisionSource source,
      Alert disconnectedAlert) {}

  private final ObjectDetectionCamera hopperFuelDetection;

  private final String objectDetectionLogRoot = "ObjectDetection";

  public ObjectDetectionVision() {
    ObjectDetectionIO hopperFuelDetectionIO;

    switch (Constants.currentMode) {
      case REAL:
        hopperFuelDetectionIO =
            new ObjectDetectionIOPhoton(VisionConstants.hopperFuelDetection.source());
        break;
      case SIM:
      case REPLAY:
      default:
        hopperFuelDetectionIO = new ObjectDetectionIO() {};
        break;
    }

    hopperFuelDetection =
        new ObjectDetectionCamera(
            hopperFuelDetectionIO,
            new ObjectDetectionIOInputsAutoLogged(),
            VisionConstants.hopperFuelDetection.source(),
            null);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    throw new UnsupportedOperationException("Unimplemented method 'simulationPeriodic'");
  }
}
