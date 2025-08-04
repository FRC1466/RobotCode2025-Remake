// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class HomeSensorIOBeamBreak implements HomeSensorIO {
  private final DigitalInput beamBreak;

  public HomeSensorIOBeamBreak(int port) {
    beamBreak = new DigitalInput(port);
  }

  @Override
  public void updateInputs(HomeSensorIOInputs inputs) {
    inputs.data = new HomeSensorIOData(!beamBreak.get());
  }
}
