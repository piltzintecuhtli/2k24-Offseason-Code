package frc.robot.subsystems.drive.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public final class DriveConstants {
  public static final double TRACK_WIDTH_X;
  public static final double TRACK_WIDTH_Y;
  public static final double MAX_LINEAR_VELOCITY;
  public static final double MAX_ANGULAR_VELOCITY;
  public static final double DRIVE_BASE_RADIUS;
  public static final SwerveDriveKinematics KINEMATICS;
  public static final String CANIVORE;
  public static final int PIGEON_2_DEVICE_ID;
  public static final Matrix<N3, N1> ODOMETRY_STANDARD_DEVIATIONS;
  public static final double DRIVER_DEADBAND;
  public static final Lock ODOMETRY_LOCK;

  static {
    switch (Constants.ROBOT) {
      case ROBOT_KRAKEN_X60:
      case ROBOT_KRAKEN_X60_PRO:
      case ROBOT_SIM:
      default:
        TRACK_WIDTH_X = Units.inchesToMeters(25.0);
        TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
        MAX_LINEAR_VELOCITY = Units.feetToMeters(17.5);
        DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2);
        MAX_ANGULAR_VELOCITY = MAX_LINEAR_VELOCITY / DRIVE_BASE_RADIUS;
        KINEMATICS =
            new SwerveDriveKinematics(
                new Translation2d[] {
                  new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                  new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
                  new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                  new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
                });
        CANIVORE = "drive";
        PIGEON_2_DEVICE_ID = 1;
        ODOMETRY_STANDARD_DEVIATIONS = VecBuilder.fill(0.0, 0.0, 0.0);
        DRIVER_DEADBAND = 0.25;
        ODOMETRY_LOCK = new ReentrantLock();
        break;
    }
  }
}
