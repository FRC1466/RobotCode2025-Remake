// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class SysIDConstants {
  public static final Velocity<VoltageUnit> translationRampRate = null;
  public static final Voltage translationStepRate = Units.Volts.of(7);
  public static final Time translationTimeout = Units.Seconds.of(5);

  /* This is in radians per second², but SysId only supports "volts per second" */
  public static final Velocity<VoltageUnit> rotationRampRate =
      Units.Volts.of(Math.PI / 6).per(Units.Second);
  /* This is in radians per second, but SysId only supports "volts" */
  public static final Voltage rotationStepRate = Units.Volts.of(Math.PI);
  public static final Time rotationTimeout = Units.Seconds.of(5);

  public static final Velocity<VoltageUnit> steerRampRate = null;
  public static final Voltage steerStepRate = Units.Volts.of(7);
  public static final Time steerTimeout = null;
}
