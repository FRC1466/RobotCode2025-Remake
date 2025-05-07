// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.sensors;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C.Port;

public class CoralSensorIOColorSensor implements CoralSensorIO {
  private final ColorSensorV3 colorSensor;

  public CoralSensorIOColorSensor(Port id) {
    colorSensor = new ColorSensorV3(id);
  }

  @Override
  public void updateInputs(CoralSensorIOInputs inputs) {
    boolean valid = colorSensor.isConnected();
    var measurement = colorSensor.getProximity();
    if (measurement > 120) {
      measurement = 10;
    } else {
      measurement = 100;
    }
    inputs.data = new CoralSensorIOData(valid ? ((double) measurement) / 100 : 0, valid);
  }
}
