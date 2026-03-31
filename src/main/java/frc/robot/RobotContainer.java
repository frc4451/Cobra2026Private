// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autonomous.AutonomousManager;
import frc.robot.bobot_state.BobotState;
import frc.robot.bobot_state.dashboard.TeleopShiftManager;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveFieldRelativePerpendicularToPoseCommand;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.ObjectDetectionVision;
import frc.robot.subsystems.vision.VisionConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.robotzgarage.frc2026.subsystems.rollers.follow.FollowRollersIO;
import org.robotzgarage.frc2026.subsystems.rollers.follow.FollowRollersIOSimTalonFX;
import org.robotzgarage.frc2026.subsystems.rollers.follow.FollowRollersIOTalonFX;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIOSimTalonFX;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIOSimTalonFXS;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIOTalonFX;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIOTalonFXSWithCANCoder;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIOTalonFXWithCANCoder;
import org.robotzgarage.frc2026.subsystems.vision.AprilTagVision;
import org.robotzgarage.frc2026.util.GarageCommandXboxController;
import org.robotzgarage.frc2026.util.TuningUtils.TuningUnit;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static final Trigger teleopActive = new Trigger(DriverStation::isTeleopEnabled);
  public static final Trigger autoActive = new Trigger(DriverStation::isAutonomousEnabled);

  // Subsystems
  // EX. private final Example example;
  private final Drive drive;
  private final Feeder feeder;
  private final Intake intake;
  private final Shooter shooter;
  private final Hood hood;

  // Vision testing
  private final AprilTagVision aprilTagVision;
  private final ObjectDetectionVision objectDetectionVision;

  // Controller
  private final GarageCommandXboxController drivercontroller = new GarageCommandXboxController(0);
  private final GarageCommandXboxController operatorcontroller = new GarageCommandXboxController(1);
  private final GarageCommandXboxController visionDebugController =
      new GarageCommandXboxController(4);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final TeleopShiftManager teleopShiftManager;

  // Shoot-on-the-move command — stored as a field so the operator binding can access its trigger
  private ShootOnTheMoveCommand sotmCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new BobotState();

    // SingleRollerIO exampleIO;
    SingleRollerIO intakeIO;
    SingleRollerIO endeffectorIO;
    SingleRollerIO hoodIO;

    FollowRollersIO shooterIO;

    SingleRollerIO towerIO;
    SingleRollerIO spindexerIO;
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, it's ALIVE!!! ALIIIIIIVVVE!!!!!
        // EX. exampleIO = new SingleRollerIOTalonFX(exampleConstants.exampleConfig);
        intakeIO = new SingleRollerIOTalonFX(IntakeConstants.intakeConfig);
        endeffectorIO =
            new SingleRollerIOTalonFXWithCANCoder(
                IntakeConstants.endeffectorConfig, IntakeConstants.cancoderConfig);

        hoodIO =
            new SingleRollerIOTalonFXSWithCANCoder(
                HoodConstants.hoodConfig, HoodConstants.cancoderConfig);

        shooterIO = new FollowRollersIOTalonFX(ShooterConstants.flywheelFollowersConfig);

        towerIO = new SingleRollerIOTalonFX(FeederConstants.towerConfig);
        spindexerIO = new SingleRollerIOTalonFX(FeederConstants.spindexerConfig);

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
        intakeIO = new SingleRollerIOSimTalonFX(IntakeConstants.intakeConfigSim);
        endeffectorIO =
            //     new SingleRollerIOSimTalonFXWithCANCoder(
            //         IntakeConstants.endeffectorConfigSim, IntakeConstants.cancoderConfig);
            new SingleRollerIOSimTalonFX(IntakeConstants.endeffectorConfigSim);

        hoodIO = new SingleRollerIOSimTalonFXS(HoodConstants.hoodConfigSim);
        shooterIO = new FollowRollersIOSimTalonFX(ShooterConstants.flywheelFollowersConfigSim);

        towerIO = new SingleRollerIOSimTalonFX(FeederConstants.towerConfigSim);
        spindexerIO = new SingleRollerIOSimTalonFX(FeederConstants.spindexerConfigSim);

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

        hoodIO = new SingleRollerIO() {};
        shooterIO = new FollowRollersIO() {};

        towerIO = new SingleRollerIO() {};
        spindexerIO = new SingleRollerIO() {};

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    feeder = new Feeder(towerIO, spindexerIO);
    intake = new Intake(intakeIO, endeffectorIO);
    hood = new Hood(hoodIO);
    shooter = new Shooter(shooterIO);
    aprilTagVision = new AprilTagVision(VisionConstants.visionSubsystemConfig);
    objectDetectionVision = new ObjectDetectionVision();
    teleopShiftManager = new TeleopShiftManager();

    // Autonomous choices
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureDriverBindings(drivercontroller);
    configureOperatorBindings(operatorcontroller);
    configureAutonomousRoutines();
    configureTuningRoutines();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureAutonomousRoutines() {
    AutonomousManager autonomousManager = new AutonomousManager(drive, shooter, intake, feeder);

    autoChooser.addOption("Himithy", autonomousManager.himithy());
    autoChooser.addOption("Himithyy", autonomousManager.himithyy());
    autoChooser.addOption("Timithy", autonomousManager.timithy());
    autoChooser.addOption("Timithyy", autonomousManager.timithyy());
    autoChooser.addOption("Timithy-1", autonomousManager.timithy_1());
    autoChooser.addOption("Himithy take 1", autonomousManager.himithyTake1());
    autoChooser.addOption("Himithy take 2", autonomousManager.himithyTake2());
    autoChooser.addOption("Dimithy-1", autonomousManager.dimithy_1());
    // autoChooser.addOption("strafe test", autonomousManager.strafeShot());

    // autoChooser.addOption("1ft BLine test", autonomousManager.bline1ft());
    // autoChooser.addOption("2ft BLine test", autonomousManager.bline2ft());
  }

  private void configureTuningRoutines() {
    if (Constants.tuningMode) {
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

      autoChooser.addOption(
          "Tune Intake Roller (Volts)",
          this.intake.rollersFeedForwardCharacterization(TuningUnit.VOLTS));
      autoChooser.addOption(
          "Tune Intake Roller (Amps)",
          this.intake.rollersFeedForwardCharacterization(TuningUnit.AMPS));

      autoChooser.addOption(
          "Tune Intake Endeffector (Volts)",
          this.intake.endeffectorFeedForwardCharacterization(TuningUnit.VOLTS));
      autoChooser.addOption(
          "Tune Intake Endeffector (Amps)",
          this.intake.endeffectorFeedForwardCharacterization(TuningUnit.AMPS));

      autoChooser.addOption(
          "Tune Shooter Flywheels (Volts)",
          this.shooter.steadyStateCharacterization(TuningUnit.VOLTS));
      autoChooser.addOption(
          "Tune Shooter Flywheels (Amps)",
          this.shooter.steadyStateCharacterization(TuningUnit.AMPS));

      autoChooser.addOption(
          "Tune Spindexer Flywheels (Volts)",
          this.feeder.kickerFeedForwardCharacterization(TuningUnit.VOLTS));
      autoChooser.addOption(
          "Tune Spindexer Flywheels (Amps)",
          this.feeder.kickerFeedForwardCharacterization(TuningUnit.AMPS));

      autoChooser.addOption(
          "Tune Tower Flywheels (Volts)",
          this.feeder.towerFeedForwardCharacterization(TuningUnit.VOLTS));
      autoChooser.addOption(
          "Tune Tower Flywheels (Amps)",
          this.feeder.towerFeedForwardCharacterization(TuningUnit.AMPS));
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureDriverBindings(GarageCommandXboxController controller) {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> BobotState.getFuelClusterVARC().getRotationTarget()));

    controller
        .a()
        .and(controller.rightBumper())
        .whileTrue(
            Commands.parallel(
                DriveCommands.joystickDriveAtAngleRobotRelative(
                    drive,
                    () -> controller.getLeftY(),
                    () -> controller.getLeftX(),
                    () -> BobotState.getFuelClusterVARC().getRotationTarget()),
                intake.runRollersVoltageOut()))
        .onFalse(intake.stop());

    sotmCommand =
        new ShootOnTheMoveCommand(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> BobotState.getShooterAlignmentVARC(),
            () -> BobotState.getShooterInterpolator());

    controller.b().and(controller.rightBumper().negate()).whileTrue(sotmCommand);

    // Use this to test shoot on the move while driving towards and aways from the hub
    // controller
    //     .b()
    //     .and(controller.rightBumper())
    //     .whileTrue(
    //         new DriveRobotRelativePerpendicularToPoseCommand(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> BobotState.getShooterAlignmentVARC().getTargetPose()));

    controller
        .x()
        .and(controller.rightBumper().negate())
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> BobotState.getClosestTraversal().getRotationTarget()));

    controller
        .x()
        .and(controller.rightBumper())
        .whileTrue(
            new DriveFieldRelativePerpendicularToPoseCommand(
                drive,
                () -> -controller.getLeftY(),
                () -> BobotState.getTrenchVARC().getTargetPose(),
                () -> BobotState.getTrenchVARC().getRotationTarget()));

    // controller
    //     .y()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> BobotState.getOutpostVARC().getRotationTarget()));

    // controller.rightTrigger().onTrue(shooter.runShooterToHub());
    // controller.rightTrigger().onFalse(shooter.stop());
    // controller.rightBumper().whileTrue(shooter.runShooterToHub()).onFalse(shooter.stop());

    // controller.leftTrigger().onTrue(shooter.runShooterVelocityTorqueCurrent());
    // controller.leftTrigger().onFalse(shooter.stop());

    // controller.leftBumper().onTrue(intake.runRollersVoltageOut());
    // controller.leftBumper().onFalse(intake.stop());

    controller.leftTrigger().onTrue(intake.runRollersVoltageOut());
    controller.leftTrigger().onFalse(intake.stop());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureOperatorBindings(GarageCommandXboxController controller) {
    // Rumble when the driver is actively aiming and within range of the SOTM target
    sotmCommand
        .aimingReady(Rotation2d.fromDegrees(8.0))
        .onTrue(controller.rumbleOnOff(0.5, 0.25, 0.25, 2));

    // Deploy the Intake as soon as Teleop starts
    teleopActive.onTrue(intake.endeffectorDeploy());

    controller.povLeft().onTrue(intake.endeffectorStart());
    controller.povUp().onTrue(intake.endeffectorSqueeze());
    controller.povRight().onTrue(intake.endeffectorLift());
    controller.povDown().onTrue(intake.endeffectorDeploy());

    controller
        .rightBumper()
        .and(BobotState.inNearAllianceZone().negate())
        .whileTrue(shooter.runShooterDropOff())
        .onFalse(shooter.stop());

    controller
        .rightBumper()
        .and(BobotState.inNearAllianceZone())
        .whileTrue(shooter.runShooterToHub())
        .onFalse(shooter.stop());

    controller.leftBumper().whileTrue(shooter.runShooterToPoop()).onFalse(shooter.stop());

    controller.a().onTrue(feeder.runFeederVoltage()).onFalse(feeder.stop());
    controller.y().onTrue(feeder.runSpindexerReverse()).onFalse(feeder.stop());
    controller.b().onTrue(hood.setHoodAngle());

    // override for shooter, for BAD things
    controller
        .leftTrigger()
        .whileTrue(shooter.runShooterVelocityTorqueCurrent())
        .onFalse(shooter.stop());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
