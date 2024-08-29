package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX topMotor;
  private final TalonFX bottomMotor;
  private final DigitalInput sensor;

  private final StatusSignal<Double> topPositionRotations;
  private final StatusSignal<Double> topVelocityRotPerSec;
  private final StatusSignal<Double> topAppliedVolts;
  private final StatusSignal<Double> topCurrentAmps;
  private final StatusSignal<Double> topTemperatureCelsius;

  private final StatusSignal<Double> bottomPositionRotations;
  private final StatusSignal<Double> bottomVelocityRotPerSec;
  private final StatusSignal<Double> bottomAppliedVolts;
  private final StatusSignal<Double> bottomCurrentAmps;
  private final StatusSignal<Double> bottomTemperatureCelsius;

  public IntakeIOTalonFX() {
    topMotor = new TalonFX(IntakeConstants.TOP_CAN_ID);
    bottomMotor = new TalonFX(IntakeConstants.BOTTOM_CAN_ID);
    sensor = new DigitalInput(IntakeConstants.SENSOR_CHANNEL);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.CURRENT_LIMIT;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    topMotor.getConfigurator().apply(config);
    bottomMotor.getConfigurator().apply(config);

    topPositionRotations = topMotor.getPosition();
    topVelocityRotPerSec = topMotor.getVelocity();
    topAppliedVolts = topMotor.getMotorVoltage();
    topCurrentAmps = topMotor.getSupplyCurrent();
    topTemperatureCelsius = topMotor.getDeviceTemp();

    bottomPositionRotations = bottomMotor.getPosition();
    bottomVelocityRotPerSec = bottomMotor.getVelocity();
    bottomAppliedVolts = bottomMotor.getMotorVoltage();
    bottomCurrentAmps = bottomMotor.getSupplyCurrent();
    bottomTemperatureCelsius = bottomMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        topPositionRotations,
        topVelocityRotPerSec,
        topAppliedVolts,
        topCurrentAmps,
        topTemperatureCelsius,
        bottomPositionRotations,
        bottomVelocityRotPerSec,
        bottomAppliedVolts,
        bottomCurrentAmps,
        bottomTemperatureCelsius);
    topMotor.optimizeBusUtilization();
    bottomMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.topPosition = Rotation2d.fromRotations(topPositionRotations.getValueAsDouble());
    inputs.topVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(topVelocityRotPerSec.getValueAsDouble());
    inputs.topAppliedVolts = topAppliedVolts.getValueAsDouble();
    inputs.topCurrentAmps = topCurrentAmps.getValueAsDouble();
    inputs.topTemperatureCelsius = topTemperatureCelsius.getValueAsDouble();

    inputs.bottomPosition = Rotation2d.fromRotations(bottomPositionRotations.getValueAsDouble());
    inputs.bottomVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(bottomCurrentAmps.getValueAsDouble());
    inputs.bottomAppliedVolts = bottomAppliedVolts.getValueAsDouble();
    inputs.bottomCurrentAmps = bottomCurrentAmps.getValueAsDouble();
    inputs.bottomTemperatureCelsius = bottomTemperatureCelsius.getValueAsDouble();

    inputs.hasNote = !sensor.get();
  }

  @Override
  public void setTopVoltage(double volts) {
    topMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setBottomVoltage(double volts) {
    bottomMotor.setControl(new VoltageOut(volts));
  }
}