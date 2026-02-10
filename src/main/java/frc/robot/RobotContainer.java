// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.prototyping.ShooterV1;
import frc.robot.subsystems.prototyping.ShooterV1Constants;
import org.robotzgarage.frc2026.subsystems.rollers.follow.FollowRollersIO;
import org.robotzgarage.frc2026.subsystems.rollers.follow.FollowRollersIOSimTalonFX;
import org.robotzgarage.frc2026.subsystems.rollers.follow.FollowRollersIOTalonFX;
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
  private final ShooterV1 shooter;
  // Controller
  private final GarageCommandXboxController controller1 = new GarageCommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // EX. exampleIO = new SingleRollerIOSimTalonFX(exampleConstants.exampleConfigSim);
        flywheelRollersIO =
            new FollowRollersIOSimTalonFX(ShooterV1Constants.flywheelFollowersConfigSim);
        feederIO = new SingleRollerIOSimTalonFX(ShooterV1Constants.feederConfigSim);
        kickerIO = new SingleRollerIOSimTalonFX(ShooterV1Constants.kickerConfigSim);
        break;

      default:
        // Replayed robot, disable IO implementations
        // EX. exampleIO = new SingleRollerIO() {};
        flywheelRollersIO = new FollowRollersIO() {};
        feederIO = new SingleRollerIO() {};
        kickerIO = new SingleRollerIO() {};
        break;
    }

    shooter = new ShooterV1(flywheelRollersIO, feederIO, kickerIO);

    // Configure the button bindings
    configureButtonBindings(controller1);

    DriverStation.silenceJoystickConnectionWarning(true);
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
