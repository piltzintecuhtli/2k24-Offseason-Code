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
  private final TalonFX acceleratorMotor;

  private final DigitalInput intakeSensor;
  private final DigitalInput middleSensor;
  private final DigitalInput finalSensor;

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

  private final StatusSignal<Double> acceleratorPositionRotations;
  private final StatusSignal<Double> acceleratorVelocityRotPerSec;
  private final StatusSignal<Double> acceleratorAppliedVolts;
  private final StatusSignal<Double> acceleratorCurrentAmps;
  private final StatusSignal<Double> acceleratorTemperatureCelsius;

  private final VoltageOut voltageOut;

  public IntakeIOTalonFX() {
    topMotor = new TalonFX(IntakeConstants.TOP_CAN_ID);
    bottomMotor = new TalonFX(IntakeConstants.BOTTOM_CAN_ID);
    acceleratorMotor = new TalonFX(IntakeConstants.ACCELERATOR_CAN_ID);

    intakeSensor = new DigitalInput(IntakeConstants.INTAKE_SENSOR_CHANNEL);
    middleSensor = new DigitalInput(IntakeConstants.MIDDLE_SENSOR_CHANNEL);
    finalSensor = new DigitalInput(IntakeConstants.FINAL_SENSOR_CHANNEL);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.CURRENT_LIMIT;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    topMotor.getConfigurator().apply(config);
    bottomMotor.getConfigurator().apply(config);
    acceleratorMotor.getConfigurator().apply(config);

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

    acceleratorPositionRotations = acceleratorMotor.getPosition();
    acceleratorVelocityRotPerSec = acceleratorMotor.getVelocity();
    acceleratorAppliedVolts = acceleratorMotor.getMotorVoltage();
    acceleratorCurrentAmps = acceleratorMotor.getSupplyCurrent();
    acceleratorTemperatureCelsius = acceleratorMotor.getDeviceTemp();

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
        bottomTemperatureCelsius,
        acceleratorPositionRotations,
        acceleratorVelocityRotPerSec,
        acceleratorAppliedVolts,
        acceleratorCurrentAmps,
        acceleratorTemperatureCelsius);
    topMotor.optimizeBusUtilization();
    bottomMotor.optimizeBusUtilization();
    acceleratorMotor.optimizeBusUtilization();

    voltageOut = new VoltageOut(0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.topPosition = Rotation2d.fromRotations(topPositionRotations.getValueAsDouble());
    inputs.topVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(topVelocityRotPerSec.getValueAsDouble());
    inputs.topAppliedVolts = topAppliedVolts.getValueAsDouble();
    inputs.topCurrentAmps = topCurrentAmps.getValueAsDouble();
    inputs.topTemperatureCelsius = topTemperatureCelsius.getValueAsDouble();

    inputs.bottomPosition = Rotation2d.fromRotations(bottomPositionRotations.getValueAsDouble());
    inputs.bottomVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(bottomCurrentAmps.getValueAsDouble());
    inputs.bottomAppliedVolts = bottomAppliedVolts.getValueAsDouble();
    inputs.bottomCurrentAmps = bottomCurrentAmps.getValueAsDouble();
    inputs.bottomTemperatureCelsius = bottomTemperatureCelsius.getValueAsDouble();

    inputs.acceleratorPosition =
        Rotation2d.fromRotations(acceleratorPositionRotations.getValueAsDouble());
    inputs.acceleratorVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(acceleratorCurrentAmps.getValueAsDouble());
    inputs.acceleratorAppliedVolts = acceleratorAppliedVolts.getValueAsDouble();
    inputs.acceleratorCurrentAmps = acceleratorCurrentAmps.getValueAsDouble();
    inputs.acceleratorTemperatureCelsius = acceleratorTemperatureCelsius.getValueAsDouble();

    inputs.intakeSensor = !intakeSensor.get();
    inputs.middleSensor = !middleSensor.get();
    inputs.finalSensor = !finalSensor.get();
  }

  @Override
  public void setTopVoltage(double volts) {
    topMotor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setBottomVoltage(double volts) {
    bottomMotor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setAcceleratorVoltage(double volts) {
    acceleratorMotor.setControl(voltageOut.withOutput(volts));
  }
}
