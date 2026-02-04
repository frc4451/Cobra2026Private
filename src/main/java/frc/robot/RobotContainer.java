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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.bobot_state.BobotState;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.VisionConstants;
import org.littletonrobotics.FieldConstants.Hub;
import org.littletonrobotics.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
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

  // private final SuperStructure superstructure;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new BobotState();
    // Superstructure subsystems
    // SingleRollerIO hoodIO;
    // SingleRollerIO turretIO;
    // SingleRollerIO shooterIO;

    // SingleRollerIO feederIO;
    // SingleRollerIO spinerIO;

    // SingleRollerIO endeffectorIO;
    // SingleRollerIO intakeIO;

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder

        // hoodIO = new SingleRollerIOTalonFX(HoodConstants.hoodConfig);
        // turretIO = new SingleRollerIOTalonFX(TurretConstants.turretConfig);
        // shooterIO = new SingleRollerIOTalonFX(ShooterConstants.shooterConfig);

        // feederIO = new SingleRollerIOTalonFX(FeederConstants.feederConfig);
        // spinerIO = new SingleRollerIOTalonFX(SpinerConstants.spinerConfig);

        // endeffectorIO = new SingleRollerIOTalonFX(EndeffectorConstants.endeffectorConfig);
        // intakeIO = new SingleRollerIOTalonFX(IntakeConstants.intakeConfig);

        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        // hoodIO = new SingleRollerIOSimTalonFX(HoodConstants.hoodConfigSim);
        // turretIO = new SingleRollerIOSimTalonFX(TurretConstants.turretConfigSim);
        // shooterIO = new SingleRollerIOSimTalonFX(ShooterConstants.shooterConfigSim);

        // feederIO = new SingleRollerIOSimTalonFX(FeederConstants.feederConfigSim);
        // spinerIO = new SingleRollerIOSimTalonFX(SpinerConstants.spinerConfigSim);

        // endeffectorIO = new SingleRollerIOSimTalonFX(EndeffectorConstants.hoodConfigSim);
        // intakeIO = new SingleRollerIOSimTalonFX(IntakeConstants.intakeConfigSim);
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

        // hoodIO = new SingleRollerIO() {};
        // turretIO = new SingleRollerIO() {};
        // shooterIO = new SingleRollerIO() {};

        // feederIO = new SingleRollerIO() {};
        // spinerIO = new SingleRollerIO() {};

        // endeffectorIO = new SingleRollerIO() {};
        // intakeIO = new SingleRollerIO() {};
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // superstructure =
    //     new SuperStructure(
    //         new ShooterComponent(hoodIO, turretIO, shooterIO),
    //         new FeederComponent(feederIO, spinerIO),
    //         new IntakeComponent(endeffectorIO, intakeIO));

    // Set up auto routines
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
    // configureOperatorBindings(operatorcontroller);

    DriverStation.silenceJoystickConnectionWarning(true);

    configureVisionDebugControls(visionDebugController);
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

    controller
        .a()
        .onTrue(
            AutoCommands.interpolationDistanceHelper(
                drive,
                new Pose2d(
                    Hub.innerCenterPoint.toTranslation2d(),
                    Hub.nearFace.getRotation().plus(Rotation2d.k180deg)),
                feetFromCenterOfHubX,
                feetFromCenterOfHubY));

    controller
        .b()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // controller.y().onTrue(drive.setPose(0.0));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller
        .x()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> BobotState.getTrenchVARC().getRotationTarget()));
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

  private void configureVisionDebugControls(GarageCommandXboxController controller) {}

  //   private void configureOperatorBindings(GarageCommandXboxController controller) {

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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
