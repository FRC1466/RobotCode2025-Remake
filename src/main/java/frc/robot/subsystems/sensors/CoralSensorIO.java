// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface CoralSensorIO {
  @AutoLog
  class CoralSensorIOInputs {
    public CoralSensorIOData data = new CoralSensorIOData(0.0, false);
  }

  record CoralSensorIOData(double distanceMeters, boolean valid) {}

  default void updateInputs(CoralSensorIOInputs inputs) {}
}
