// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  // Motor IDs
  public static final int masterMotorId = 17;
  public static final int followerMotorId = 16;

  // Current Limits
  public static final double supplyCurrentLimit = 60.0;
  public static final double statorCurrentLimit = 120.0;

  // PID Constants
  public static final double kP = 1.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kG = 0.0;
  public static final double kS = 0.0;

  // Motion Magic Constants
  public static final double accelerationConstraint = 8;
  public static final double accelerationConstraintAlgae = 6;
  public static final double accelerationConstraintDown = 6;

  public static final double velocityConstraint = 3;
  public static final double velocityConstraintAlgae = 2.8;

  public static final double cascadeCarriageMultiplier = 2.0;
  public static final double pulleyRadiusInches = 1.0;
  public static final double elevatorGearRatio = 3.0 * 4.0;

  public static final double elevatorRotationsToMeters(int rotations) {
    return (rotations / elevatorGearRatio) * cascadeCarriageMultiplier * (2 * Math.PI) * Units.inchesToMeters(pulleyRadiusInches);
  }
  
  public static final double elevatorMetersToRotations(int meters) {
    return (meters / (cascadeCarriageMultiplier * (2 * Math.PI) * Units.inchesToMeters(pulleyRadiusInches))) * elevatorGearRatio;
  }
}
