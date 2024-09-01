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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ModuleIOSim implements ModuleIO {
  private DCMotorSim driveSim;
  private DCMotorSim turnSim;

  private final PIDController driveFeedback;
  private final PIDController turnFeedback;

  private final Rotation2d turnAbsoluteInitPosition;

  private double driveAppliedVolts;
  private double turnAppliedVolts;

  public ModuleIOSim() {
    driveSim =
        new DCMotorSim(
            ModuleConstants.DRIVE_MOTOR_CONFIG,
            ModuleConstants.DRIVE_GEAR_RATIO,
            ModuleConstants.DRIVE_MOMENT_OF_INERTIA);
    turnSim =
        new DCMotorSim(
            ModuleConstants.TURN_MOTOR_CONFIG,
            ModuleConstants.TURN_GEAR_RATIO,
            ModuleConstants.TURN_MOMENT_OF_INERTIA);

    driveFeedback =
        new PIDController(
            ModuleConstants.DRIVE_KP.get(),
            0.0,
            ModuleConstants.DRIVE_KD.get(),
            Constants.LOOP_PERIOD_SECS);
    turnFeedback =
        new PIDController(
            ModuleConstants.TURN_KP.get(),
            0.0,
            ModuleConstants.TURN_KD.get(),
            Constants.LOOP_PERIOD_SECS);

    turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);

    driveAppliedVolts = 0.0;
    turnAppliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(Constants.LOOP_PERIOD_SECS);
    turnSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new Rotation2d[] {inputs.drivePosition};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};

    inputs.drivePosition = Rotation2d.fromRadians(driveSim.getAngularPositionRad());
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.turnAbsolutePosition =
        new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    inputs.driveVelocityError = driveFeedback.getVelocityError();
    inputs.turnPositionError = turnFeedback.getPositionError();
  }

  @Override
  public void setDriveVelocitySetpoint(double velocityRadPerSec, double feedForward) {
    driveAppliedVolts =
        MathUtil.clamp(driveFeedback.calculate(velocityRadPerSec) + feedForward, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnPositionSetpoint(Rotation2d setpointPosition) {
    turnAppliedVolts =
        MathUtil.clamp(turnFeedback.calculate(setpointPosition.getRadians()), -12.0, 12.0);
    driveSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void stop() {}
}
