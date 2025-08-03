// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.elevator.Elevator.ElevatorProfile;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  default void updateInputs(ElevatorIOInputs inputs) {}

  @AutoLog
  class ElevatorIOInputs {
    public double elevatorPositionMeters;
    public double elevatorVelocityMetersPerSec;
    public double elevatorAccelerationMetersPerSecSquared;

    public double elevatorAppliedVolts;
    public double elevatorSupplyCurrentAmps;
    public double elevatorStatorCurrentAmps;

    public double elevatorMasterMotorTemp;
    public double elevatorFollowerMotorTemp;
  }

  default void setTargetPosition(double positionInMeters) {}

  default void resetElevatorPosition(double positionInMeters) {}

  default void setDutyCycle(double dutyCycle) {}

  default void setNeutralMode(NeutralModeValue neutralMode) {}

  default void setMotionProfileConstraints(ElevatorProfile elevatorProfile) {}

  default void setPID(double kP, double kI, double kD) {}
}
