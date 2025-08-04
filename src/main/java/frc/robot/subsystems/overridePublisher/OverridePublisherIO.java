// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.overridePublisher;

import org.littletonrobotics.junction.AutoLog;

public interface OverridePublisherIO {
  @AutoLog
  public static class OverridePublisherIOInputs {
    public boolean OverrideAll = false;
    public boolean ReefOverride = false;
    public boolean AlgaeOverride = false;
    public boolean StationOverride = false;
  }

  default void updateInputs(OverridePublisherIOInputs inputs) {}
}
