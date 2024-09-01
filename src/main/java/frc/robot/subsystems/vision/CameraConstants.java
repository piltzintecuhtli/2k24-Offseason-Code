package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;

public class CameraConstants {
  public static final double BLINK_TIME = 0.067;

  public static class Limelight3Constants {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(62.5);
    public static final double VERTICAL_FOV = Units.degreesToRadians(48.9);
    public static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    public static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
  }

  public static class Limelight3GConstants {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(82.0);
    public static final double VERTICAL_FOV = Units.degreesToRadians(46.2);
    public static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.0026;
    public static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.0015;
  }

  public static class ArducamOV9281 {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(62.5);
    public static final double VERTICAL_FOV = Units.degreesToRadians(48.9);
    public static final double MULTITAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    public static final double AVERAGE_BEST_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
  }

  public static class RobotCameras {}
}
