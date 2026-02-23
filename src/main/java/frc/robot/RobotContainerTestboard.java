// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.bobot_state.BobotState;
import frc.robot.subsystems.vision.ObjectDetectionVision;
import frc.robot.subsystems.vision.VisionConstants;
import org.robotzgarage.frc2026.subsystems.vision.AprilTagVision;
import org.robotzgarage.frc2026.util.GarageCommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainerTestboard {
  // Subsystems
  // EX. private final Example example;

  // Vision testing
  private final AprilTagVision aprilTagVision;
  private final ObjectDetectionVision objectDetectionVision;

  // Controller
  private final GarageCommandXboxController drivercontroller = new GarageCommandXboxController(0);
  private final GarageCommandXboxController operatorcontroller = new GarageCommandXboxController(1);
  private final GarageCommandXboxController tunercontroller = new GarageCommandXboxController(2);
  private final GarageCommandXboxController visionDebugController =
      new GarageCommandXboxController(4);

  // Dashboard inputs
  //   private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainerTestboard() {
    new BobotState();
    aprilTagVision = new AprilTagVision(VisionConstants.visionSubsystemConfig);
    objectDetectionVision = new ObjectDetectionVision();

    DriverStation.silenceJoystickConnectionWarning(true);
    // configureVisionDebugControls(visionDebugController);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.get();
    return Commands.none();
  }
}
