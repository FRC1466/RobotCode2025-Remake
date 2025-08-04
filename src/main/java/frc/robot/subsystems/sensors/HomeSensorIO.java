// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface HomeSensorIO {
  @AutoLog
  class HomeSensorIOInputs {
    public HomeSensorIOData data = new HomeSensorIOData(false);
  }

  record HomeSensorIOData(boolean broken) {}

  default void updateInputs(HomeSensorIOInputs inputs) {}
}
