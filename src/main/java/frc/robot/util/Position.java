// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * Immutable container for a combined mechanism pose: - Slapdown angle - Wrist angle - Elevator
 * height (meters)
 *
 * <p>Designed for storing preset positions and later commanding all mechanisms together.
 */
public record Position(
    Rotation2d slapdownAngle, Rotation2d wristAngle, double elevatorHeightMeters) {

  public Position {
    if (slapdownAngle == null) throw new IllegalArgumentException("slapdownAngle null");
    if (wristAngle == null) throw new IllegalArgumentException("wristAngle null");
    if (Double.isNaN(elevatorHeightMeters) || Double.isInfinite(elevatorHeightMeters)) {
      throw new IllegalArgumentException("elevatorHeightMeters invalid");
    }
  }

  public static Position of(Rotation2d slapdown, Rotation2d wrist, double elevatorMeters) {
    return new Position(slapdown, wrist, elevatorMeters);
  }

  public static Position fromDegrees(double slapdownDeg, double wristDeg, double elevatorMeters) {
    return of(
        Rotation2d.fromDegrees(slapdownDeg), Rotation2d.fromDegrees(wristDeg), elevatorMeters);
  }

  public static Position fromRadians(double slapdownRad, double wristRad, double elevatorMeters) {
    return of(new Rotation2d(slapdownRad), new Rotation2d(wristRad), elevatorMeters);
  }

  public Position withSlapdown(Rotation2d newSlapdown) {
    return new Position(newSlapdown, wristAngle, elevatorHeightMeters);
  }

  public Position withWrist(Rotation2d newWrist) {
    return new Position(slapdownAngle, newWrist, elevatorHeightMeters);
  }

  public Position withElevator(double newElevatorMeters) {
    return new Position(slapdownAngle, wristAngle, newElevatorMeters);
  }

  public double elevatorHeightInInches() {
    return Units.metersToInches(elevatorHeightMeters);
  }

  @Override
  public String toString() {
    return "Position(slapdownDeg=%.2f, wristDeg=%.2f, elevator=%.3fm)"
        .formatted(slapdownAngle.getDegrees(), wristAngle.getDegrees(), elevatorHeightMeters);
  }
}
