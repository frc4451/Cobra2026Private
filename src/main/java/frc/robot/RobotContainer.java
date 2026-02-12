// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.subsystems.prototyping.IntakeV1;
import frc.robot.subsystems.prototyping.IntakeV1Constants;
import org.littletonrobotics.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIOSimTalonFX;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIOTalonFX;
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
  private final IntakeV1 intake;

  private final Drive drive;

  // Controller
  private final GarageCommandXboxController drivercontroller = new GarageCommandXboxController(0);
  private final GarageCommandXboxController operatorcontroller = new GarageCommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new BobotState();

    // SingleRollerIO exampleIO;
    SingleRollerIO intakeIO;
    SingleRollerIO endeffectorIO;
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, it's ALIVE!!! ALIIIIIIVVVE!!!!!
        // EX. exampleIO = new SingleRollerIOTalonFX(exampleConstants.exampleConfig);
        intakeIO = new SingleRollerIOTalonFX(IntakeV1Constants.intakeConfig);
        endeffectorIO = new SingleRollerIOTalonFX(IntakeV1Constants.endeffectorConfig);

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
        intakeIO = new SingleRollerIOSimTalonFX(IntakeV1Constants.intakeConfigSim);
        endeffectorIO = new SingleRollerIOSimTalonFX(IntakeV1Constants.endeffectorConfigSim);

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
        intakeIO = new SingleRollerIO() {};
        endeffectorIO = new SingleRollerIO() {};

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    intake = new IntakeV1(intakeIO, endeffectorIO);
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

    // Configure the button bindings
    configureButtonBindings(drivercontroller);

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
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

    // controller
    //     .b()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> Rotation2d.kZero));

    // controller.y().onTrue(drive.setPose(0.0));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // controller
    //     .x()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> BobotState.getTrenchVARC().getRotationTarget()));

    // // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> Rotation2d.kZero));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(
    //                             drive.getWheelOdometry().getTranslation(), Rotation2d.kZero)),
    //                 drive)
    //             .ignoringDisable(true));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings(GarageCommandXboxController controller) {
    // EX. controller.a().whileTrue(example.runshooterpercent(0.5));
    //     // Intake Component controlls
    //     controller
    //         .leftBumper()
    //         .onTrue(superstructure.endeffectorIn()); // when left bumper is pressed, move
    // endeffector in
    //     controller
    //         .rightBumper()
    //         .onTrue(superstructure.endeffectorOut()); // when right bumper is pressed, move
    // endeffector
    //     // out
    //     controller
    //         .rightBumper() // when right bumper is pressed
    //         .and(superstructure.isEndeffectorOut()) // if endeffector is out
    //         .whileTrue(superstructure.runIntakeCommand()); // run intake, while right bumper is
    // held

    //     // Feeder Component controlls
    //     controller
    //         .a()
    //         .whileTrue(superstructure.runShooterCommand())
    //         .whileFalse(superstructure.runShooterCommand(0.0));

    //     controller
    //         .a()
    //         .and(superstructure.isShooterReady())
    //         .whileTrue(superstructure.runFeederCommand())
    //         .whileTrue(superstructure.runSpinerCommand())
    //         .whileFalse(superstructure.runFeederCommand(0.0))
    //         .whileFalse(superstructure.runSpinerCommand(0.0));

    // Shooter Component controlls

    //   }
    controller.a().onTrue(intake.runIntakePercentOut());
    // controller.b().onTrue(intake.runIntakeTorqueCurrent());
    // controller.x().onTrue(intake.runIntakeVoltageOut());
    controller.y().onTrue(intake.stop());

    // controller.povUp().onTrue(intake.setEndeffectorGoal());
    // controller.povDown().onTrue(intake.endeffectorHome());
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
