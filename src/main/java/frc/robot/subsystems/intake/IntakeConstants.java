package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;

public class IntakeConstants {
  public static final int TOP_CAN_ID;
  public static final int BOTTOM_CAN_ID;
  public static final int ACCELERATOR_CAN_ID;

  public static final int INTAKE_SENSOR_CHANNEL;
  public static final int MIDDLE_SENSOR_CHANNEL;
  public static final int FINAL_SENSOR_CHANNEL;

  public static final double CURRENT_LIMIT;

  public static final DCMotor TOP_MOTOR;
  public static final DCMotor BOTTOM_MOTOR;
  public static final DCMotor ACCELERATOR_MOTOR;

  public static final double TOP_GEAR_RATIO;
  public static final double BOTTOM_GEAR_RATIO;
  public static final double ACCELERATOR_GEAR_RATIO;

  static {
    switch (Constants.ROBOT) {
      case ROBOT_KRAKEN_X60:
      case ROBOT_SIM:
      default:
        TOP_CAN_ID = 1;
        BOTTOM_CAN_ID = 2;
        ACCELERATOR_CAN_ID = 3;
        INTAKE_SENSOR_CHANNEL = 0;
        MIDDLE_SENSOR_CHANNEL = 0;
        FINAL_SENSOR_CHANNEL = 0;
        CURRENT_LIMIT = 40.0;
        TOP_MOTOR = DCMotor.getKrakenX60(1);
        BOTTOM_MOTOR = DCMotor.getKrakenX60(1);
        ACCELERATOR_MOTOR = DCMotor.getKrakenX60(1);
        TOP_GEAR_RATIO = 1.0;
        BOTTOM_GEAR_RATIO = 1.0;
        ACCELERATOR_GEAR_RATIO = 1.0;
        break;
    }
  }
}
