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

package frc.robot.subsystems.drive.gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.drive.SparkMaxOdometryThread;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import java.util.OptionalDouble;
import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIONavX implements GyroIO {
  private final AHRS navX = new AHRS();

  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  private final Alert disconnectedAlert =
      new Alert("Pigeon is disconnected, check CAN bus.", AlertType.ERROR);

  public GyroIONavX() {
    yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(() -> OptionalDouble.of(navX.getAngle()));
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    boolean connected = navX.isConnected();
    disconnectedAlert.set(!connected);

    inputs.connected = connected;
    inputs.yawPosition = Rotation2d.fromDegrees(navX.getAngle());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(navX.getRate());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
