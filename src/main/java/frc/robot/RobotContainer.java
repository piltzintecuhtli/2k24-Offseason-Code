// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.drive.Drive;
import frc.robot.subsystems.drive.drive.DriveConstants;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.vision.CameraConstants.RobotCameras;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  // Dashboard Inputs
  private LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case ROBOT_KRAKEN_X60:
        case ROBOT_KRAKEN_X60_FOC:
          // Snapback, instantiate hardware IO implementations
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(0),
                  new ModuleIOTalonFX(1),
                  new ModuleIOTalonFX(2),
                  new ModuleIOTalonFX(3));
          vision =
              new Vision(RobotCameras.LIMELIGHT_LEFT_ARDUCAM, RobotCameras.LIMELIGHT_RIGHT_ARDUCAM);

          break;
        case ROBOT_SIM:
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          vision = new Vision();
      }
    }

    // Instantiate missing subsystems
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (vision == null) {
      vision = new Vision();
    }

    // Configure autobuilder.
    AutoBuilder.configureHolonomic(
        RobotState::getRobotPose,
        RobotState::resetRobotPose,
        () -> DriveConstants.KINEMATICS.toChassisSpeeds(drive.getModuleStates()),
        drive::runVelocity,
        new HolonomicPathFollowerConfig(
            DriveConstants.MAX_LINEAR_VELOCITY,
            DriveConstants.DRIVE_BASE_RADIUS,
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        drive);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "LocalADStarAK/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("LocalADStarAK/Trajectory Setpoint", targetPose);
        });

    // Configure auto choices.
    autoChooser = new LoggedDashboardChooser<>("Auto Routines");
    autoChooser.addDefaultOption("None", AutoRoutines.none());

    // Configure the button bindings
    configureButtonBindings();

    // Configure shuffleboard
    Shuffleboard.getTab("Autonomous")
        .add("Autonomous Mode", autoChooser.getSendableChooser())
        .withPosition(0, 0)
        .withSize(2, 2);
    Shuffleboard.getTab("Teleoperated")
        .addNumber("Hood Offset", RobotState::getSpeakerAngleCompensation)
        .withPosition(0, 0)
        .withSize(1, 1);
    Shuffleboard.getTab("Teleoperated")
        .addNumber("Flywheel Offset", RobotState::getSpeakerFlywheelCompensation)
        .withPosition(0, 1)
        .withSize(1, 1);
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    driver.y().onTrue(CompositeCommands.resetHeading(drive));
  }

  public void robotPeriodic() {
    RobotState.periodic(drive.getRotation(), drive.getYawVelocity(), drive.getFieldRelativeVelocity(), drive.getModulePositions(), vision.getCameras(), vision.getValidTarget(), vision.getPrimaryVisionPoses(), vision.getSecondaryVisionPoses(), vision.getFrameTimestamps());
  }

  public Command getAutonomousCommand() {
    return AutoRoutines.ampSideSixPiece(drive);
  }
}
