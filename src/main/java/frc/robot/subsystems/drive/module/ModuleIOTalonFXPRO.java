// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.drive.DriveConstants;
import frc.robot.subsystems.drive.drive.PhoenixOdometryThread;
import frc.robot.subsystems.drive.module.ModuleConstants.ModuleConfig;
import java.util.Queue;

public class ModuleIOTalonFXPRO implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;
  private final StatusSignal<Double> driveTemp;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;
  private final StatusSignal<Double> turnTemp;

  private final StatusSignal<Double> driveVelocityError;
  private final StatusSignal<Double> turnPositionError;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  private final TalonFXConfiguration driveConfig;
  private final TalonFXConfiguration turnConfig;

  private final VelocityVoltage velocityControl;
  private final PositionVoltage positionControl;
  private final VoltageOut voltageControl;
  private final NeutralOut neutralControl;

  public ModuleIOTalonFXPRO(ModuleConfig moduleConfig) {
    driveTalon = new TalonFX(moduleConfig.drive(), DriveConstants.CANIVORE);
    turnTalon = new TalonFX(moduleConfig.turn(), DriveConstants.CANIVORE);
    cancoder = new CANcoder(moduleConfig.encoder(), DriveConstants.CANIVORE);
    absoluteEncoderOffset = moduleConfig.absoluteEncoderOffset();

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();
    driveTemp = driveTalon.getDeviceTemp();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getSupplyCurrent();
    turnTemp = turnTalon.getDeviceTemp();

    driveVelocityError = driveTalon.getClosedLoopError();
    turnPositionError = turnTalon.getClosedLoopError();

    driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.DRIVE_CURRENT_LIMIT;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.TURN_CURRENT_LIMIT;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    turnTalon.getConfigurator().apply(turnConfig);
    setTurnBrakeMode(true);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    BaseStatusSignal.setUpdateFrequencyForAll(
        ModuleConstants.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveTemp,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent,
        turnTemp,
        driveVelocityError,
        turnPositionError);

    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();

    velocityControl = new VelocityVoltage(0.0);
    positionControl = new PositionVoltage(0.0);
    voltageControl = new VoltageOut(0.0);
    neutralControl = new NeutralOut();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .map(
                (Double value) ->
                    Rotation2d.fromRotations(value / ModuleConstants.DRIVE_GEAR_RATIO))
            .toArray(Rotation2d[]::new);
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map(
                (Double value) -> Rotation2d.fromRotations(value / ModuleConstants.TURN_GEAR_RATIO))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();

    inputs.drivePosition =
        Rotation2d.fromRotations(
            drivePosition.getValueAsDouble() / ModuleConstants.DRIVE_GEAR_RATIO);
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble())
            / ModuleConstants.DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
    inputs.driveTempCelcius = driveTemp.getValueAsDouble();

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnPosition.getValueAsDouble() / ModuleConstants.TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / ModuleConstants.TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();
    inputs.turnTempCelcius = turnTemp.getValueAsDouble();

    inputs.driveVelocityError = driveVelocityError.getValueAsDouble();
    inputs.turnPositionError = turnPositionError.getValueAsDouble();
  }

  @Override
  public void setDriveVelocitySetpoint(double velocityRadsPerSec, double feedForward) {
    driveTalon.setControl(
        velocityControl
            .withVelocity(velocityRadsPerSec)
            .withFeedForward(feedForward)
            .withEnableFOC(true));
  }

  @Override
  public void setTurnPositionSetpoint(Rotation2d position) {
    turnTalon.setControl(positionControl.withPosition(position.getRotations()).withEnableFOC(true));
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(voltageControl.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(voltageControl.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveConfig.Slot0.kP = kP;
    driveConfig.Slot0.kI = kI;
    driveConfig.Slot0.kD = kD;
    driveTalon.getConfigurator().apply(driveConfig, 0.01);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnConfig.Slot0.kP = kP;
    turnConfig.Slot0.kI = kI;
    turnConfig.Slot0.kD = kD;
    turnTalon.getConfigurator().apply(turnConfig, 0.01);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.Clockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }

  @Override
  public void stop() {
    driveTalon.setControl(neutralControl);
    turnTalon.setControl(neutralControl);
  }
}
