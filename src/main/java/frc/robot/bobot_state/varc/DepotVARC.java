// package frc.robot.bobot_state.varc;

// import static edu.wpi.first.units.Units.Meters;

// import org.littletonrobotics.AllianceFlipUtil;
// import org.littletonrobotics.FieldConstants.Depot;
// import org.robotzgarage.frc2026.math.varc.TargetAngleTracker;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.units.measure.Distance;
// import frc.robot.bobot_state.BobotState;


/*broken*/




// public class DepotVARC extends TargetAngleTracker{
//     public void update() {

//         // public static final Translation3d depotpoint =
//         // new Translation3d(
//         //     aprilTagFieldLayout.getTagPose(26).get().getX() + width / 2.0,
//         //     fieldWidth / 2.0,
//         //     innerHeight);

//         Pose2d robotPose = BobotState.getGlobalPose();
//         Translation2d depotLocation =
// AllianceFlipUtil.apply(Depot.depotCenter.toTranslation2d());
//         Pose2d depotPose =

//         this.distance = Meters.of(depotLocation.getDistance(robotPose.getTranslation()));

//         this.rotationTarget = depotCenter
//             .relativeTo(robotPose)
//             .getTranslation()
//             .getAngle()
//             .plus(robotPose.getRotation());
//     }
// }
