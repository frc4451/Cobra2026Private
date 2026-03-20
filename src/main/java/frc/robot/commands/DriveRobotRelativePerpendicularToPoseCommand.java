package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.math.distance_calc.PoseUtils;

public class DriveRobotRelativePerpendicularToPoseCommand extends Command {
  private final ProfiledPIDController parallelController =
      new ProfiledPIDController(5.0, 0.0, 0.3, new TrapezoidProfile.Constraints(4.0, 4.0));

  private final ProfiledPIDController angleController =
      new ProfiledPIDController(5.0, 0.0, 0.3, new TrapezoidProfile.Constraints(8.0, 20.0));

  private final Drive drive;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final DoubleSupplier perpendicularInput;

  private double perpendicularError = 0;

  public DriveRobotRelativePerpendicularToPoseCommand(
      Drive drive, DoubleSupplier perpendicularInput, Supplier<Pose2d> targetPoseSupplier) {
    addRequirements(drive);

    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
    this.perpendicularInput = perpendicularInput;
  }

  public DriveRobotRelativePerpendicularToPoseCommand withJoystickRumble(
      DoubleSupplier rumbleDistance, Command rumbleCommand) {
    atSetpoint(rumbleDistance).onTrue(Commands.deferredProxy(() -> rumbleCommand));

    return this;
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getGlobalPose();
    Pose2d targetPose = targetPoseSupplier.get();
    Logger.recordOutput("Commands/" + getName() + "/targetPose", targetPose);

    Rotation2d desiredTheta = targetPose.getRotation().plus(Rotation2d.kPi);

    perpendicularError = PoseUtils.getPerpendicularError(robotPose, targetPose);
    Logger.recordOutput("Commands/" + getName() + "/PerpendicularError", perpendicularError);

    double parallelError = PoseUtils.getParallelError(robotPose, targetPose);
    Logger.recordOutput("Commands/" + getName() + "/ParallelError", parallelError);

    double thetaError = robotPose.getRotation().minus(desiredTheta).getRadians();
    Logger.recordOutput("Commands/" + getName() + "/ThetaError", thetaError);

    double parallelSpeed = parallelController.calculate(-parallelError, 0);
    parallelSpeed = !parallelController.atSetpoint() ? parallelSpeed : 0;

    double angularSpeed = angleController.calculate(thetaError, 0);
    angularSpeed = !angleController.atSetpoint() ? angularSpeed : 0;

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            perpendicularInput.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec(),
            parallelSpeed,
            angularSpeed);

    drive.runVelocity(speeds);

    Logger.recordOutput(
        "Commands/" + getName() + "/ParallelAtSetpoint", parallelController.atSetpoint());
    Logger.recordOutput("Commands/" + getName() + "/AngleAtSetpoint", angleController.atSetpoint());
  }

  @Override
  public void end(boolean interrupt) {
    parallelController.reset(0);
    angleController.reset(0);
  }

  public Trigger atSetpoint(DoubleSupplier distance) {
    return new Trigger(
        () ->
            parallelController.atSetpoint()
                && angleController.atSetpoint()
                && Math.abs(perpendicularError) < distance.getAsDouble());
  }
}
