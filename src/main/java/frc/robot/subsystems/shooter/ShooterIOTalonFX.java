package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  private final StatusSignal<Double> topPositionRotations;
  private final StatusSignal<Double> topVelocityRotPerSec;
  private final StatusSignal<Double> topAppliedVolts;
  private final StatusSignal<Double> topCurrentAmps;
  private final StatusSignal<Double> topTemperatureCelsius;

  private final StatusSignal<Double> bottomPositionRotations;
  private final StatusSignal<Double> bottomVelocityRotPerSec;
  private final StatusSignal<Double> bottomAppliedVolts;
  private final StatusSignal<Double> bottomCurrentAmps;
  private final StatusSignal<Double> bottomTemperatureCelcius;

  private final StatusSignal<Double> topVelocityErrorRotationsPerSecond;
  private final StatusSignal<Double> bottomVelocityErrorRotationsPerSecond;

  private final TalonFXConfiguration topConfig;
  private final TalonFXConfiguration bottomConfig;


  private final NeutralOut neutralControl;
  private final VoltageOut voltageControl;
  private final MotionMagicVelocityVoltage topProfiledVelocityControl;
  private final MotionMagicVelocityVoltage bottomProfiledVelocityControl; 

  public ShooterIOTalonFX() {
    topMotor = new TalonFX(ShooterConstants.TOP_CAN_ID);
    bottomMotor = new TalonFX(ShooterConstants.BOTTOM_CAN_ID);

    topConfig = new TalonFXConfiguration();
    bottomConfig = new TalonFXConfiguration();

    topConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    topConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //pid
    topConfig.Slot0.kP = ShooterConstants.KP.get();
    topConfig.Slot0.kD = ShooterConstants.KD.get();
    topConfig.Slot0.kS = ShooterConstants.KS.get();
    topConfig.Slot0.kV = ShooterConstants.KV.get();
    topConfig.MotionMagic.MotionMagicAcceleration = 
        Units.radiansToRotations(
          ShooterConstants.MAX_ACCELERATION_RADIANS_PER_SECEOND_SQUARED.get());
    
    topMotor.getConfigurator().apply(topConfig);

    bottomConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    bottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //pid
    bottomConfig.Slot0.kP = ShooterConstants.KP.get();
    bottomConfig.Slot0.kD = ShooterConstants.KD.get();
    bottomConfig.Slot0.kS = ShooterConstants.KS.get();
    bottomConfig.Slot0.kV = ShooterConstants.KV.get();
    bottomConfig.MotionMagic.MotionMagicAcceleration = 
      Units.radiansToRotations(
        ShooterConstants.MAX_ACCELERATION_RADIANS_PER_SECEOND_SQUARED.get());

    bottomMotor.getConfigurator().apply(bottomConfig);

    topPositionRotations = topMotor.getPosition();
    topVelocityRotPerSec = topMotor.getVelocity();
    topAppliedVolts = topMotor.getMotorVoltage();
    topCurrentAmps = topMotor.getSupplyCurrent();
    topTemperatureCelsius = topMotor.getDeviceTemp();

    bottomPositionRotations = bottomMotor.getPosition();
    bottomVelocityRotPerSec = bottomMotor.getVelocity();
    bottomAppliedVolts = bottomMotor.getMotorVoltage();
    bottomCurrentAmps = bottomMotor.getSupplyCurrent();
    bottomTemperatureCelcius = bottomMotor.getDeviceTemp();

    topVelocityErrorRotationsPerSecond = topMotor.getClosedLoopError();
    bottomVelocityErrorRotationsPerSecond = bottomMotor.getClosedLoopError();

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
        bottomTemperatureCelcius);

    topMotor.optimizeBusUtilization();
    bottomMotor.optimizeBusUtilization();

    neutralControl = new NeutralOut();
    voltageControl = new VoltageOut(0.0);
    topProfiledVelocityControl = new MotionMagicVelocityVoltage(0.0);
    bottomProfiledVelocityControl = new MotionMagicVelocityVoltage(0.0);

  }
  
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.topPosition = Rotation2d.fromRotations(topPositionRotations.getValueAsDouble());
    inputs.topVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(topVelocityRotPerSec.getValueAsDouble());
    inputs.topAppliedVolts = topAppliedVolts.getValueAsDouble();
    inputs.topCurrentAmps = topCurrentAmps.getValueAsDouble();
    inputs.topTemperatureCelsius = topTemperatureCelsius.getValueAsDouble();

    inputs.bottomPosition = Rotation2d.fromRotations(bottomPositionRotations.getValueAsDouble());
    inputs.bottomVelocityRadPerSec = Units
        .rotationsPerMinuteToRadiansPerSecond(bottomVelocityRotPerSec.getValueAsDouble());
    inputs.bottomAppliedVolts = bottomAppliedVolts.getValueAsDouble();
    inputs.bottomCurrentAmps = bottomCurrentAmps.getValueAsDouble();
    inputs.bottomTemperatureCelsius = bottomTemperatureCelcius.getValueAsDouble();

    inputs.topVelocityErrorRadiansPerSecond = Units
        .rotationsToRadians(topVelocityErrorRotationsPerSecond.getValueAsDouble()
            / ShooterConstants.TOP_GEAR_RATIO);
    inputs.bottomVelocityErrorRadiansPerSecond = Units
        .rotationsToRadians(bottomVelocityErrorRotationsPerSecond.getValueAsDouble()
            / ShooterConstants.BOTTOM_GEAR_RATIO);
  }

  @Override
  public void setTopPID(double P, double I, double D) {
    topConfig.Slot0.kP = P;
    topConfig.Slot0.kI = I;
    topConfig.Slot0.kD = D;
    topMotor.getConfigurator().apply(topConfig, 0.001);
  }

  @Override
  public void setBottomPID(double P, double I, double D) {
    bottomConfig.Slot0.kP = P;
    bottomConfig.Slot0.kI = I;
    bottomConfig.Slot0.kD = D;
    bottomMotor.getConfigurator().apply(topConfig, 0.001);
  }

  @Override
  public void setVoltage(double volts) {
    topMotor.setControl(new VoltageOut(volts));
    bottomMotor.setControl(new VoltageOut(volts));
  }
  
  @Override
  public void stop() {
    topMotor.setControl(neutralControl);
    bottomMotor.setControl(neutralControl);
  }
  
}
