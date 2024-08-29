package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;

public class IntakeConstants {
  public static final int TOP_CAN_ID;
  public static final int BOTTOM_CAN_ID;
  public static final int SENSOR_CHANNEL;
  public static final double CURRENT_LIMIT;
  public static final DCMotor TOP_MOTOR;
  public static final DCMotor BOTTOM_MOTOR;
  public static final double GEAR_RATIO;
  
  static {
    switch (Constants.ROBOT) {
      case ROBOT_KRAKEN_X60:
      case ROBOT_KRAKEN_X60_FOC:
      case ROBOT_SIM:
      default:
        TOP_CAN_ID = 1;
        BOTTOM_CAN_ID = 2;
        SENSOR_CHANNEL = 0;
        CURRENT_LIMIT = 40.0;
        TOP_MOTOR = DCMotor.getKrakenX60(1);
        BOTTOM_MOTOR = DCMotor.getKrakenX60(1);
            GEAR_RATIO = 1.0;
        break;
    }
  }
}
