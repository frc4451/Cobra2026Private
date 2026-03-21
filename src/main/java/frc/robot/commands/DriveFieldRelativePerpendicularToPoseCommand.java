package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.robotzgarage.frc2026.math.distance_calc.PoseUtils;

public class DriveFieldRelativePerpendicularToPoseCommand extends Command {

  private final Drive drive;
  private final DoubleSupplier perpendicularInput;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final Supplier<Rotation2d> desiredHeadingSupplier;

  private final ProfiledPIDController parallelController =
      new ProfiledPIDController(3.0, 0.0, 0.3, new TrapezoidProfile.Constraints(4.0, 4.0));

  private final ProfiledPIDController angleController =
      new ProfiledPIDController(3.0, 0.0, 0.3, new TrapezoidProfile.Constraints(8.0, 20.0));

  // Pose where we want to align the robot
  private Pose2d targetPose = new Pose2d();
  // Robot's target facing direction
  private Rotation2d desiredHeading = Rotation2d.kZero;

  public DriveFieldRelativePerpendicularToPoseCommand(
      Drive drive,
      DoubleSupplier perpendicularInput,
      Supplier<Pose2d> targetPoseSupplier,
      Supplier<Rotation2d> desiredHeadingSupplier) {
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
    this.desiredHeadingSupplier = desiredHeadingSupplier;
    this.perpendicularInput = perpendicularInput;

    parallelController.setTolerance(Units.inchesToMeters(0.25));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(Units.degreesToRadians(0.5));

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    targetPose = targetPoseSupplier.get();
    desiredHeading = desiredHeadingSupplier.get();
    parallelController.reset(0);
    angleController.reset(drive.getGlobalPose().getRotation().minus(desiredHeading).getRadians());
  }

  @Override
  public void execute() {
    double parallelError = PoseUtils.getParallelError(drive.getGlobalPose(), targetPose);
    double thetaError = drive.getGlobalPose().getRotation().minus(desiredHeading).getRadians();

    double parallelSpeed = parallelController.calculate(parallelError, 0);
    parallelSpeed = !parallelController.atSetpoint() ? parallelSpeed : 0;

    double angularSpeed = angleController.calculate(thetaError, 0);
    angularSpeed = !angleController.atSetpoint() ? angularSpeed : 0;

    double xAxisSpeed = perpendicularInput.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec();
    Translation2d targetTranslation = new Translation2d(xAxisSpeed, parallelSpeed);

    Translation2d fieldTranslation = targetTranslation.rotateBy(targetPose.getRotation());
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldTranslation.getX(),
            fieldTranslation.getY(),
            angularSpeed,
            drive.getGlobalPose().getRotation());

    drive.runVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
  }
}
