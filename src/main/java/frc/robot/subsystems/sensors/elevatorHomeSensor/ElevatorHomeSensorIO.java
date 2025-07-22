// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.sensors.elevatorHomeSensor;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorHomeSensorIO {
  @AutoLog
  class ElevatorHomeSensorIOInputs {
    public ElevatorHomeSensorIOData data = new ElevatorHomeSensorIOData(false, false);
  }

  record ElevatorHomeSensorIOData(boolean atBottom, boolean valid) {}

  default void updateInputs(ElevatorHomeSensorIOInputs inputs) {}
}
