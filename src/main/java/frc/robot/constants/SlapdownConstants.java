// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import frc.robot.util.LoggedTunableNumber;

public class SlapdownConstants {
  // Motor IDs
  public static final int motorId = 14;

  // Current Limits
  public static final double supplyCurrentLimit = 30.0;
  public static final double statorCurrentLimit = 80.0;

  // PID Constants
  public static final LoggedTunableNumber kP = new LoggedTunableNumber("Wrist/kP", 8);
  public static final LoggedTunableNumber kI = new LoggedTunableNumber("Wrist/kI", 0.0);
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("Wrist/kD", 0.0);
  public static final LoggedTunableNumber kG = new LoggedTunableNumber("Wrist/kG", 0.0);
  public static final LoggedTunableNumber kS = new LoggedTunableNumber("Wrist/kS", 0.0);

  // Motion Magic Constants, in mechanism rotations
  public static final double accelerationConstraint = 80;
  public static final double velocityConstraint = 35;

  // Gear Ratio
  public static final double slapdownReduction = (58.0 / 10.0) * (58.0 / 18.0);

  // Conversion Functions
  public static final double wristRotationsToRadians(double rotations) {
    return rotations * (2 * Math.PI) / slapdownReduction;
  }

  public static final double wristRadiansToRotations(double radians) {
    return radians * slapdownReduction / (2 * Math.PI);
  }

  // Setpoints (in Radians)
  public static final LoggedTunableNumber stowedPosition =
      new LoggedTunableNumber("Wrist/StowedPosition", 0.0);

  public static final LoggedTunableNumber intakePosition =
      new LoggedTunableNumber("Wrist/IntakePosition", 2.269);
}
