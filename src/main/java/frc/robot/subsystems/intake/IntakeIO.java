package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
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

    public boolean hasNote = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {
  }

  public default void setTopVoltage(double volts) {
  }

  public default void setBottomVoltage(double volts) {}
}
