package frc.robot.commands;

import com.choreo.lib.Choreo;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.drive.Drive;
import frc.robot.subsystems.drive.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;

public class CompositeCommands {

  public static final Command resetHeading(Drive drive) {
    return Commands.runOnce(
            () -> {
              drive.setPose(
                  new Pose2d(
                      RobotState.getRobotPose().getTranslation(),
                      AllianceFlipUtil.apply(new Rotation2d())));
              RobotState.resetRobotPose(
                  new Pose2d(
                      RobotState.getRobotPose().getTranslation(),
                      AllianceFlipUtil.apply(new Rotation2d())));
            })
        .ignoringDisable(true);
  }

  public static final Command getChoreoCommand(Drive drive, String trajectory) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              drive.setPose(
                  AllianceFlipUtil.apply(Choreo.getTrajectory(trajectory).getInitialPose()));
              RobotState.resetRobotPose(
                  AllianceFlipUtil.apply(Choreo.getTrajectory(trajectory).getInitialPose()));
            }),
        Choreo.choreoSwerveCommand(
            Choreo.getTrajectory(trajectory),
            RobotState::getRobotPose,
            new PIDController(DriveConstants.AUTO_X_KP.get(), 0.0, DriveConstants.AUTO_X_KD.get()),
            new PIDController(DriveConstants.AUTO_Y_KP.get(), 0.0, DriveConstants.AUTO_Y_KD.get()),
            new PIDController(
                DriveConstants.AUTO_THETA_KP.get(), 0.0, DriveConstants.AUTO_THETA_KD.get()),
            (ChassisSpeeds speeds) -> drive.runVelocity(speeds),
            () ->
                DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get().equals(Alliance.Red),
            drive));
  }
}
