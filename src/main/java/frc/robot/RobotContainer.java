// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.bobot_state.BobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.prototyping.ShooterV1;
import frc.robot.subsystems.prototyping.ShooterV1Constants;
import frc.robot.subsystems.vision.VisionConstants;
import org.littletonrobotics.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.robotzgarage.frc2026.subsystems.rollers.follow.FollowRollersIO;
import org.robotzgarage.frc2026.subsystems.rollers.follow.FollowRollersIOSimTalonFX;
import org.robotzgarage.frc2026.subsystems.rollers.follow.FollowRollersIOTalonFX;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIOSimTalonFX;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIOTalonFX;
import org.robotzgarage.frc2026.subsystems.vision.Vision;
import org.robotzgarage.frc2026.util.GarageCommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  // EX. private final Example example;
  private final ShooterV1 shooter;

  private final Drive drive;

  // Vision testing
  private final Vision vision;

  // Controller
  private final GarageCommandXboxController drivercontroller = new GarageCommandXboxController(0);
  private final GarageCommandXboxController operatorcontroller = new GarageCommandXboxController(1);

  private final GarageCommandXboxController visionDebugController =
      new GarageCommandXboxController(4);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new BobotState();

    // SingleRollerIO exampleIO;
    FollowRollersIO flywheelRollersIO;
    SingleRollerIO feederIO;
    SingleRollerIO kickerIO;
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, it's ALIVE!!! ALIIIIIIVVVE!!!!!
        // EX. exampleIO = new SingleRollerIOTalonFX(exampleConstants.exampleConfig);
        flywheelRollersIO = new FollowRollersIOTalonFX(ShooterV1Constants.flywheelFollowersConfig);
        feederIO = new SingleRollerIOTalonFX(ShooterV1Constants.feederShooterConfig);
        kickerIO = new SingleRollerIOTalonFX(ShooterV1Constants.kickerShooterConfig);

        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // EX. exampleIO = new SingleRollerIOSimTalonFX(exampleConstants.exampleConfigSim);
        flywheelRollersIO =
            new FollowRollersIOSimTalonFX(ShooterV1Constants.flywheelFollowersConfigSim);
        feederIO = new SingleRollerIOSimTalonFX(ShooterV1Constants.feederConfigSim);
        kickerIO = new SingleRollerIOSimTalonFX(ShooterV1Constants.kickerConfigSim);

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        // EX. exampleIO = new SingleRollerIO() {};
        flywheelRollersIO = new FollowRollersIO() {};
        feederIO = new SingleRollerIO() {};
        kickerIO = new SingleRollerIO() {};

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    shooter = new ShooterV1(flywheelRollersIO, feederIO, kickerIO);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings

    configureDriverBindings(drivercontroller);

    vision = new Vision(VisionConstants.visionSubsystemConfig);

    SmartDashboard.putNumber("CameraTest/PoseXMeters", 0);
    SmartDashboard.putNumber("CameraTest/PoseYMeters", 0);
    SmartDashboard.putNumber("CameraTest/PoseRotationDeg", 0);

    visionDebugController
        .back()
        .onTrue(
            new InstantCommand(
                () -> {
                  Pose2d desiredPose =
                      new Pose2d(
                          new Translation2d(
                              SmartDashboard.getNumber("CameraTest/PoseXMeters", 0),
                              SmartDashboard.getNumber("CameraTest/PoseYMeters", 0)),
                          Rotation2d.fromDegrees(
                              SmartDashboard.getNumber("CameraTest/PoseRotationDeg", 0)));
                  drive.setPose(desiredPose);
                }));
    configureDriverBindings(drivercontroller);
    DriverStation.silenceJoystickConnectionWarning(true);
    // configureVisionDebugControls(visionDebugController);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings(GarageCommandXboxController controller) {

    // controller.x().whileTrue(shooter.runShooterFlywheelPercentOut());
    // controller.x().whileTrue(shooter.runFeederPercentOutput());
    // controller.x().whileTrue(shooter.runAllPercent());
    controller.y().whileTrue(shooter.runAllVoltage());
  }

  private void configureDriverBindings(GarageCommandXboxController controller) {
    LoggedTunableNumber feetFromCenterOfHubX = new LoggedTunableNumber("FeetFromHubCenterX", 0);
    LoggedTunableNumber feetFromCenterOfHubY = new LoggedTunableNumber("FeetFromHubCenterY", 0);

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> controller.getRightX()));

    // controller
    //     .a()
    //     .onTrue(
    //         AutoCommands.interpolationDistanceHelper(
    //             drive,
    //             new Pose2d(
    //                 Hub.innerCenterPoint.toTranslation2d(),
    //                 Hub.nearFace.getRotation().plus(Rotation2d.k180deg)),
    //             feetFromCenterOfHubX,
    //             feetFromCenterOfHubY));

    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> BobotState.getClosestDropoff().getRotationTarget()));

    controller
        .b()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> BobotState.getHubVARC().getRotationTarget()));

    controller
        .x()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> BobotState.getClosestTraversal().getRotationTarget()));

    controller
        .y()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> BobotState.getOutpostVARC().getRotationTarget()));

    // controller.a().whileTrue(shooter.runShooterFlywheelVelocityTorqueCurrent());
    // controller.x().whileTrue(shooter.runShooterFlywheelTorqueCurrent());
    // controller.a().whileTrue(shooter.runFeederVolts());
    controller.a().whileTrue(shooter.runShooterTuning());
    controller.b().whileTrue(shooter.stop());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
