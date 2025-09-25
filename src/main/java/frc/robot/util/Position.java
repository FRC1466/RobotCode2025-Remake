// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Immutable container for a mechanism pose consisting of: - Elevator height (meters) - Main pivot
 * angle
 */
public record Position(double elevatorHeightMeters, Rotation2d pivotAngle) {

  public Position {}

  // Preferred simple factories
  public static Position of(double elevatorMeters, Rotation2d pivot) {
    return new Position(elevatorMeters, pivot);
  }

  public static Position fromDegrees(double elevatorMeters, double pivotDeg) {
    return of(elevatorMeters, Rotation2d.fromDegrees(pivotDeg));
  }

  public static Position fromRadians(double elevatorMeters, double pivotRad) {
    return of(elevatorMeters, new Rotation2d(pivotRad));
  }
}
