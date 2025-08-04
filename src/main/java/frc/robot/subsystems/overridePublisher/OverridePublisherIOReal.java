// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.overridePublisher;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OverridePublisherIOReal implements OverridePublisherIO {
  public OverridePublisherIOReal() {
    SmartDashboard.putBoolean("Overrides/ReefOverride", false);
    SmartDashboard.putBoolean("Overrides/AlgaeOverride", false);
    SmartDashboard.putBoolean("Overrides/StationOverride", false);
  }

  @Override
  public void updateInputs(OverridePublisherIOInputs inputs) {
    inputs.ReefOverride = SmartDashboard.getBoolean("Overrides/ReefOverride", false);
    inputs.AlgaeOverride = SmartDashboard.getBoolean("Overrides/AlgaeOverride", false);
    inputs.StationOverride = SmartDashboard.getBoolean("Overrides/StationOverride", false);
  }
}
