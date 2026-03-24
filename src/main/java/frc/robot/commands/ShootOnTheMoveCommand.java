package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.bobot_state.BobotState;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Field-relative drive command that continuously tracks a shoot-on-the-move rotation target.
 *
 * <p>The angle controller feeds the target's instantaneous angular velocity as a goal velocity into
 * the ProfiledPIDController so it can follow a moving SOTM aim setpoint without lag or overshoot.
 * The rotation target supplier (e.g. {@code
 * BobotState.getShooterAlignmentVARC()::getRotationTarget}) already incorporates the lookahead
 * correction computed by {@code TargetInterpolator}.
 */
public class ShootOnTheMoveCommand extends Command {
  private static final double DEADBAND = 0.1;

  // Maximum angular rate (rad/s) the robot is allowed to sweep around the hub during SOTM.
  // Mirrors the 6328 approach: the triangle formed by (robot, hub, projected position) is solved
  // with the sine rule to derive the maximum linear speed that satisfies this polar constraint.
  // Lower values = tighter speed cap near the hub. Default 0.6 rad/s matches 6328's value.
  private static final LoggedTunableNumber MAX_POLAR_VELOCITY_RAD_PER_SEC =
      new LoggedTunableNumber("ShootOnTheMoveCommand/MaxPolarVelocityRadPerSec", 0.6);

  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final Supplier<Rotation2d> rotationSupplier;

  // Shared angle controller. Gains are read from DriveCommands tunables so both the
  // regular joystickDriveAtAngle command and this command stay in sync when gains are tuned.
  private final ProfiledPIDController angleController =
      new ProfiledPIDController(
          DriveCommands.ANGLE_KP.get(),
          0.0,
          DriveCommands.ANGLE_KD.get(),
          new TrapezoidProfile.Constraints(
              DriveCommands.ANGLE_MAX_VELOCITY.get(), DriveCommands.ANGLE_MAX_ACCELERATION.get()));

  // Stores the rotation target from the previous loop so we can finite-difference it into an
  // angular velocity. That velocity is fed as the goal velocity to the ProfiledPIDController,
  // which lets the controller plan to *follow* a moving target rather than stop at it.
  private double prevTargetRad = 0.0;

  /**
   * @param drive the drive subsystem
   * @param xSupplier field-relative X input (forward positive, pre-negated from joystick)
   * @param ySupplier field-relative Y input (left positive, pre-negated from joystick)
   * @param rotationSupplier target heading supplier (e.g. VARC::getRotationTarget)
   */
  public ShootOnTheMoveCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;

    // Wrap-around so the controller always takes the short path around 0/360.
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(Units.degreesToRadians(1.0));

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Seed prevTargetRad to the current target so the first-loop velocity estimate is 0
    // (not a huge spike from 0 → wherever the target happens to be).
    prevTargetRad = rotationSupplier.get().getRadians();
    // Reset the controller to the robot's actual heading so it doesn't try to unwind a
    // stale accumulated error from a previous activation.
    angleController.reset(BobotState.getGlobalPose().getRotation().getRadians());
  }

  @Override
  public void execute() {
    // Step 1: Read and shape the joystick inputs
    // Apply a circular deadband (magnitude of the 2D vector, not per-axis) so the
    // direction stays stable even near the deadband boundary.
    double x = xSupplier.getAsDouble();
    double y = ySupplier.getAsDouble();
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);

    // Preserve the direction separately so we can re-apply it after clamping magnitude.
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    // Square the magnitude for finer low-speed control while keeping full-speed reachable.
    linearMagnitude *= linearMagnitude;

    // Step 2: Geometry-based speed limit (hub shots only)
    // At close range, even a small robot velocity can sweep a large angle around the
    // hub during fuel flight, making the lookahead aim angle unstable or flip directions.
    // To compat this, cap the robot's linear speed so it can sweep at most
    // MAX_POLAR_VELOCITY_RAD_PER_SEC
    // around the hub during the ball's time of flight (TOF).
    //
    // We model this as a triangle R-H-P where:
    //   R = robot's current position
    //   H = hub center
    //   P = robot's projected position after traveling for one TOF
    //
    // The three interior angles are:
    //   robotAngle    (at R) = angle between the direction we're driving and the line R→H
    //   hubAngle      (at H) = maxPolarVelocity × TOF  — the max arc we're allowed to sweep
    //   lookaheadAngle (at P) = π − robotAngle − hubAngle  (triangle angle sum)
    //
    // By the sine rule:  R→P / sin(hubAngle) = R→H / sin(lookaheadAngle)
    // → maxDistance = hubDistance × sin(hubAngle) / sin(lookaheadAngle)
    // → maxVelocity = maxDistance / TOF
    //
    // If lookaheadAngle <= 0 the triangle is degenerate (driving away from hub).
    if (BobotState.getCurrentFieldZone() == BobotState.FieldZone.NEAR_ALLIANCE
        && linearMagnitude > 0.0) {
      Distance hubDistance = BobotState.getHubVARC().getDistance();

      // Look up ball TOF at the current (non-lookahead) distance from the interpolation table.
      Optional<Time> maybeTOF = BobotState.getHubInterpolator().getNaiveTOF(hubDistance);

      if (maybeTOF.isPresent()) {
        double tof = maybeTOF.get().in(Seconds);

        // Angle at H: how many radians the hub "sees" the robot sweep in one TOF.
        double hubAngle = MAX_POLAR_VELOCITY_RAD_PER_SEC.get() * tof;

        // Angle at R: difference between where we're driving and the direction toward the hub.
        // Using the absolute value because we only care about the magnitude of the deviation.
        Translation2d robotToHub =
            BobotState.getHubVARC()
                .getTargetPose()
                .getTranslation()
                .minus(BobotState.getGlobalPose().getTranslation());
        double robotAngle =
            Math.abs(
                MathUtil.angleModulus(robotToHub.getAngle().minus(linearDirection).getRadians()));

        // Remaining angle at P (triangle must sum to π).
        double lookaheadAngle = Math.PI - robotAngle - hubAngle;
        if (lookaheadAngle > 0) {
          // Sine rule → max distance the robot may travel in one TOF.
          double maxDistanceMeters =
              hubDistance.in(Meters) * Math.sin(hubAngle) / Math.sin(lookaheadAngle);
          // Convert to a normalized magnitude (0–1) relative to the drive's top speed.
          double maxMagnitude = (maxDistanceMeters / tof) / drive.getMaxLinearSpeedMetersPerSec();
          linearMagnitude = Math.min(linearMagnitude, maxMagnitude);
        }
      }
    }

    if (BobotState.getCurrentFieldZone() != BobotState.FieldZone.NEAR_ALLIANCE
        && linearMagnitude > 0.0) {
      Distance dropoffDistance = BobotState.getHubVARC().getDistance();

      // Look up ball TOF at the current (non-lookahead) distance from the interpolation table.
      Optional<Time> maybeTOF = BobotState.getDropoffInterpolator().getNaiveTOF(dropoffDistance);

      if (maybeTOF.isPresent()) {
        double tof = maybeTOF.get().in(Seconds);

        // Angle at H: how many radians the hub "sees" the robot sweep in one TOF.
        double dropoffAngle = MAX_POLAR_VELOCITY_RAD_PER_SEC.get() * tof;

        // Angle at R: difference between where we're driving and the direction toward the hub.
        // Using the absolute value because we only care about the magnitude of the deviation.
        Translation2d robotToDropoff =
            BobotState.getClosestDropoff()
                .getTargetPose()
                .getTranslation()
                .minus(BobotState.getGlobalPose().getTranslation());
        double robotAngle =
            Math.abs(
                MathUtil.angleModulus(
                    robotToDropoff.getAngle().minus(linearDirection).getRadians()));

        // Remaining angle at P (triangle must sum to π).
        double lookaheadAngle = Math.PI - robotAngle - dropoffAngle;
        if (lookaheadAngle > 0) {
          // Sine rule → max distance the robot may travel in one TOF.
          double maxDistanceMeters =
              dropoffDistance.in(Meters) * Math.sin(dropoffAngle) / Math.sin(lookaheadAngle);
          // Convert to a normalized magnitude (0–1) relative to the drive's top speed.
          double maxMagnitude = (maxDistanceMeters / tof) / drive.getMaxLinearSpeedMetersPerSec();
          linearMagnitude = Math.min(linearMagnitude, maxMagnitude);
        }
      }
    }

    // ── Step 3: Reconstruct the velocity vector from clamped magnitude + original direction ─
    Translation2d linearVelocity =
        new Pose2d(Translation2d.kZero, linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
            .getTranslation();

    // ── Step 4: SOTM angle tracking with velocity feed-forward ──────────────────────────────
    // The rotation target moves every loop as the robot drives and the lookahead updates.
    // A plain ProfiledPIDController would plan to *stop* at the goal, causing lag when the
    // setpoint is moving. Instead we estimate how fast the target is rotating this loop
    // (finite difference over 20 ms) and pass that as the goal velocity, so the controller
    // plans to *follow* the target at its current rate.
    double currentTarget = rotationSupplier.get().getRadians();
    double targetVelocity = MathUtil.angleModulus(currentTarget - prevTargetRad) / 0.02;
    prevTargetRad = currentTarget;

    double omega =
        angleController.calculate(
            BobotState.getGlobalPose().getRotation().getRadians(),
            // TrapezoidProfile.State(position, velocity) — the velocity here is the feed-forward.
            new TrapezoidProfile.State(currentTarget, targetVelocity));

    // ── Step 5: Convert to field-relative chassis speeds and send to drive ──────────────────
    // On Red alliance the driver's "forward" is flipped 180°, so rotate the field frame.
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega,
            isFlipped
                ? BobotState.getGlobalPose().getRotation().plus(Rotation2d.kPi)
                : BobotState.getGlobalPose().getRotation()));

    // ── Logging ─────────────────────────────────────────────────────────────────────────────
    Logger.recordOutput(
        "Commands/" + getName() + "/TargetAngleDeg", Units.radiansToDegrees(currentTarget));
    Logger.recordOutput(
        "Commands/" + getName() + "/AngleErrorDeg",
        Units.radiansToDegrees(angleController.getPositionError()));
    Logger.recordOutput("Commands/" + getName() + "/AimingReady", angleController.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
  }

  /** Trigger issued while the robot is aimed within {@code tolerance} of the target. */
  public Trigger aimingReady(Rotation2d tolerance) {
    return new Trigger(() -> Math.abs(angleController.getPositionError()) < tolerance.getRadians());
  }
}
