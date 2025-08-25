// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public class DifferentialWristPivotConstants {
  // Tunable PID for pivot and wrist
  public static final LoggedTunableNumber kP_Pivot =
      new LoggedTunableNumber("DiffWristPivot/kP_Pivot", 8.0);
  public static final LoggedTunableNumber kI_Pivot =
      new LoggedTunableNumber("DiffWristPivot/kI_Pivot", 0.0);
  public static final LoggedTunableNumber kD_Pivot =
      new LoggedTunableNumber("DiffWristPivot/kD_Pivot", 0.0);

  public static final LoggedTunableNumber kP_Wrist =
      new LoggedTunableNumber("DiffWristPivot/kP_Wrist", 8.0);
  public static final LoggedTunableNumber kI_Wrist =
      new LoggedTunableNumber("DiffWristPivot/kI_Wrist", 0.0);
  public static final LoggedTunableNumber kD_Wrist =
      new LoggedTunableNumber("DiffWristPivot/kD_Wrist", 0.0);

  // Motion constraints (rad/s, rad/s^2) – if needed by IOs later
  public static final double velocityConstraintRadPerSec = Units.degreesToRadians(180);
  public static final double accelerationConstraintRadPerSec2 = Units.degreesToRadians(720);

  // Safe limits
  public static final double pivotMinRadians = Units.degreesToRadians(-60);
  public static final double pivotMaxRadians = Units.degreesToRadians(120);
  public static final double wristMinRadians = Units.degreesToRadians(-540);
  public static final double wristMaxRadians = Units.degreesToRadians(540);

  // Default setpoints
  public static final Rotation2d defaultPivotAngle = Rotation2d.fromRadians(0.0);
  public static final Rotation2d defaultWristAngle = Rotation2d.fromRadians(0.0);
}
