package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public Rotation2d topPosition = new Rotation2d();
    public double topVelocityRadPerSec = 0.0;
    public double topAppliedVolts = 0.0;
    public double topCurrentAmps = 0.0;
    public double topTemperatureCelsius = 0.0;

    public Rotation2d bottomPosition = new Rotation2d();
    public double bottomVelocityRadPerSec = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double bottomCurrentAmps = 0.0;
    public double bottomTemperatureCelsius = 0.0;

    public double topVelocityErrorRadiansPerSecond = 0.0;
    public double bottomVelocityErrorRadiansPerSecond = 0.0;
  }
  
  public default void updateInputs(ShooterIOInputs inputs) {}
  
  public default void setTopVelocitySetpoint(double setpointVelocityRadiansPerSecond) {}

  public default void setBottomVelocitySetpoint(double setpointVelocityRadiansPerSecond) {}

  public default void setVoltage(double volts) {}

  public default void setTopPID(double kP, double kI, double kD) {}

  public default void setBottomPID(double kP, double kI, double kD) {}

  public default void setTopFeedforward(double kS, double kV, double kA) {}

  public default void setBottomFeedforward(double kS, double kV, double kA) {}

  public default void setTopProfile(double maxAccelerationRadiansPerSecondSquared) {}

  public default void setBottomProfile(double maxAccelerationRadiansPerSecondSquared) {}

  public default boolean atSetpoint() {
    return false;
  }
  
  public default void stop() {}
}
