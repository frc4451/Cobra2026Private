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
import frc.robot.subsystems.prototyping.IntakeV1;
import frc.robot.subsystems.prototyping.IntakeV1Constants;
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
  // Controller
  private final GarageCommandXboxController controller1 = new GarageCommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // SingleRollerIO exampleIO;
    SingleRollerIO intakeIO;
    SingleRollerIO endeffectorIO;
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, it's ALIVE!!! ALIIIIIIVVVE!!!!!
        // EX. exampleIO = new SingleRollerIOTalonFX(exampleConstants.exampleConfig);
        intakeIO = new SingleRollerIOTalonFX(IntakeV1Constants.intakeConfig);
        endeffectorIO = new SingleRollerIOTalonFX(IntakeV1Constants.endeffectorConfig);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // EX. exampleIO = new SingleRollerIOSimTalonFX(exampleConstants.exampleConfigSim);
        intakeIO = new SingleRollerIOSimTalonFX(IntakeV1Constants.intakeConfigSim);
        endeffectorIO = new SingleRollerIOSimTalonFX(IntakeV1Constants.endeffectorConfigSim);
        break;

      default:
        // Replayed robot, disable IO implementations
        // EX. exampleIO = new SingleRollerIO() {};
        intakeIO = new SingleRollerIO() {};
        endeffectorIO = new SingleRollerIO() {};
        break;
    }

    intake = new IntakeV1(intakeIO, endeffectorIO);

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
    // EX. controller.a().whileTrue(example.runshooterpercent(0.5));

    controller.a().onTrue(intake.runIntakePercentOut());
    controller.b().onTrue(intake.runIntakeTorqueCurrent());
    controller.x().onTrue(intake.runIntakeVoltageOut());
    controller.y().onTrue(intake.stop());

    controller.povUp().onTrue(intake.setEndeffectorGoal());
    controller.povDown().onTrue(intake.endeffectorHome());
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
