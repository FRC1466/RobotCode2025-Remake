// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public ElevatorIOData data = new ElevatorIOData(false, false, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  record ElevatorIOData(
      boolean elevatorMasterMotorConnected,
      boolean elevatorFollowerMotorConnected,
      double elevatorPositionMeters,
      double elevatorVelocityMetersPerSec,
      double elevatorAccelerationMetersPerSecSquared,
      double elevatorAppliedVolts,
      double elevatorSupplyCurrentAmps,
      double elevatorStatorCurrentAmps,
      double elevatorMasterMotorTemp,
      double elevatorFollowerMotorTemp) {}

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setTargetPosition(double positionInMeters) {}

  default void resetElevatorPosition(double positionInMeters) {}

  default void setDutyCycle(double dutyCycle) {}

  default void setNeutralMode(NeutralModeValue neutralMode) {}

  default void setPID(double kP, double kI, double kD) {}
}
