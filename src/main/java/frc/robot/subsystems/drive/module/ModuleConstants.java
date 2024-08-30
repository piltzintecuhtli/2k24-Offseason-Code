package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import lombok.Builder;

public class ModuleConstants {
  public static final LoggedTunableNumber WHEEL_RADIUS =
      new LoggedTunableNumber("Drive/Wheel Radius");
  public static final LoggedTunableNumber DRIVE_KS = new LoggedTunableNumber("Drive/Drive Ks");
  public static final LoggedTunableNumber DRIVE_KV = new LoggedTunableNumber("Drive/Drive Kv");
  public static final LoggedTunableNumber DRIVE_KP = new LoggedTunableNumber("Drive/Drive Kp");
  public static final LoggedTunableNumber DRIVE_KD = new LoggedTunableNumber("Drive/Drive Kd");
  public static final LoggedTunableNumber TURN_KP = new LoggedTunableNumber("Drive/Turn Kp");
  public static final LoggedTunableNumber TURN_KD = new LoggedTunableNumber("Drive/Turn Kd");

  public static final ModuleConfigCANCoder frontLeftConfigCANCoder;
  public static final ModuleConfigCANCoder frontRightConfigCANCoder;
  public static final ModuleConfigCANCoder rearLeftConfigCANCoder;
  public static final ModuleConfigCANCoder rearRightConfigCANCoder;

  public static final double ODOMETRY_FREQUENCY;
  public static final double OUT_OF_SYNC_THRESHOLD;
  public static final double DRIVE_GEAR_RATIO;
  public static final double TURN_GEAR_RATIO;
  public static final int DRIVE_CURRENT_LIMIT;
  public static final int TURN_CURRENT_LIMIT;
  public static final int CAN_TIMEOUT;
  public static final double NOMINAL_VOLTAGE;
  public static final int VELOCITY_MEASUREMENT_PERIOD;
  public static final int VELOCITY_AVERAGE_DEPTH;
  public static final double DRIVE_MOMENT_OF_INERTIA;
  public static final double TURN_MOMENT_OF_INERTIA;
  public static final DCMotor DRIVE_MOTOR_CONFIG;
  public static final DCMotor TURN_MOTOR_CONFIG;

  static {
    ODOMETRY_FREQUENCY = 250.0;
    OUT_OF_SYNC_THRESHOLD = Units.degreesToRadians(30.0);
    DRIVE_GEAR_RATIO = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
    TURN_GEAR_RATIO = 150.0 / 7.0;
    DRIVE_CURRENT_LIMIT = 40;
    TURN_CURRENT_LIMIT = 30;
    CAN_TIMEOUT = 250;
    NOMINAL_VOLTAGE = 12.0;
    VELOCITY_MEASUREMENT_PERIOD = 10;
    VELOCITY_AVERAGE_DEPTH = 2;
    DRIVE_MOMENT_OF_INERTIA = 0.025;
    TURN_MOMENT_OF_INERTIA = 0.004;
    switch (Constants.ROBOT) {
      case ROBOT_KRAKEN_X60:
        WHEEL_RADIUS.initDefault(Units.inchesToMeters(2.0));
        DRIVE_KS.initDefault(0.063566);
        DRIVE_KV.initDefault(0.11799);
        DRIVE_KP.initDefault(0.13);
        DRIVE_KD.initDefault(0.0);
        TURN_KP.initDefault(9.0);
        TURN_KD.initDefault(0.0);
        frontLeftConfigCANCoder =
            new ModuleConfigCANCoder(1, 2, 20, Rotation2d.fromRadians(2.2304080655857224));
        frontRightConfigCANCoder =
            new ModuleConfigCANCoder(3, 4, 21, Rotation2d.fromRadians(-1.4910293258248433));
        rearLeftConfigCANCoder =
            new ModuleConfigCANCoder(5, 6, 22, Rotation2d.fromRadians(-0.2132233295161041));
        rearRightConfigCANCoder =
            new ModuleConfigCANCoder(7, 8, 23, Rotation2d.fromRadians(-1.4327380558851888));
        DRIVE_MOTOR_CONFIG = DCMotor.getNeoVortex(1);
        TURN_MOTOR_CONFIG = DCMotor.getNeoVortex(1);
        break;
      case ROBOT_SIM:
        DRIVE_MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
        TURN_MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
        WHEEL_RADIUS.initDefault(Units.inchesToMeters(2.0));
        DRIVE_KS.initDefault(-0.0081157);
        DRIVE_KV.initDefault(0.12821);
        DRIVE_KP.initDefault(0.039024);
        DRIVE_KD.initDefault(0.0);
        TURN_KP.initDefault(10.0);
        TURN_KD.initDefault(0.0);
        frontLeftConfigCANCoder = null;
        frontRightConfigCANCoder = null;
        rearLeftConfigCANCoder = null;
        rearRightConfigCANCoder = null;
        break;
      default:
        DRIVE_MOTOR_CONFIG = DCMotor.getKrakenX60(1);
        TURN_MOTOR_CONFIG = DCMotor.getKrakenX60(1);
        frontLeftConfigCANCoder = null;
        frontRightConfigCANCoder = null;
        rearLeftConfigCANCoder = null;
        rearRightConfigCANCoder = null;
        break;
    }
  }

  @Builder
  public record ModuleConfigCANCoder(
      int drive, int turn, int encoder, Rotation2d absoluteEncoderOffset) {}
}
