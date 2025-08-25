// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public class TurretConstants {
  // Motor IDs
  public static final int motorId = 14;

  // Current Limits
  public static final double supplyCurrentLimit = 30.0;
  public static final double statorCurrentLimit = 80.0;

  // PID Constants
  public static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", 5);
  public static final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI", 0.0);
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", 0.0);
  public static final LoggedTunableNumber kG = new LoggedTunableNumber("Turret/kG", 0.0);
  public static final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", 0.0);

  // Motion Magic Constants, in rotations
  public static final double accelerationConstraint = Units.rotationsToRadians(10);
  public static final double velocityConstraint = Units.rotationsToRadians(2);

  // Gear Ratio
  public static final double turretReduction = (58.0 / 10.0) * (58.0 / 18.0);

  // Conversion Functions
  public static final double turretRotationsToRadians(double rotations) {
    return rotations * (2 * Math.PI) / turretReduction;
  }

  public static final double turretRadiansToRotations(double radians) {
    return radians * turretReduction / (2 * Math.PI);
  }

  // Setpoints (in Radians)
  public static final LoggedTunableNumber stowedPosition =
      new LoggedTunableNumber("Turret/StowedPosition", 0.0);
}
