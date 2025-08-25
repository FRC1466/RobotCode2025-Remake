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
 * Immutable container for a combined mechanism pose: - Coral slapdown angle - Algae slapdown angle
 * - Elevator height (meters) - Main pivot angle - Differential pivot angle - Differential wrist
 * angle
 *
 * <p>Designed for storing preset positions and later commanding all mechanisms together.
 */
public record Position(
    Rotation2d coralSlapdownAngle,
    Rotation2d algaeSlapdownAngle,
    double elevatorHeightMeters,
    Rotation2d pivotAngle,
    Rotation2d differentialPivotAngle,
    Rotation2d differentialWristAngle) {

  public Position {
    if (coralSlapdownAngle == null) throw new IllegalArgumentException("coralSlapdownAngle null");
    if (algaeSlapdownAngle == null) throw new IllegalArgumentException("algaeSlapdownAngle null");
    if (pivotAngle == null) throw new IllegalArgumentException("pivotAngle null");
    if (differentialPivotAngle == null)
      throw new IllegalArgumentException("differentialPivotAngle null");
    if (differentialWristAngle == null)
      throw new IllegalArgumentException("differentialWristAngle null");
    if (Double.isNaN(elevatorHeightMeters) || Double.isInfinite(elevatorHeightMeters)) {
      throw new IllegalArgumentException("elevatorHeightMeters invalid");
    }
  }

  // Full builders
  public static Position of(
      Rotation2d coralSlapdown,
      Rotation2d algaeSlapdown,
      double elevatorMeters,
      Rotation2d pivot,
      Rotation2d differentialPivot,
      Rotation2d differentialWrist) {
    return new Position(
        coralSlapdown, algaeSlapdown, elevatorMeters, pivot, differentialPivot, differentialWrist);
  }

  public static Position fromDegrees(
      double coralSlapdownDeg,
      double algaeSlapdownDeg,
      double elevatorMeters,
      double pivotDeg,
      double differentialPivotDeg,
      double differentialWristDeg) {
    return of(
        Rotation2d.fromDegrees(coralSlapdownDeg),
        Rotation2d.fromDegrees(algaeSlapdownDeg),
        elevatorMeters,
        Rotation2d.fromDegrees(pivotDeg),
        Rotation2d.fromDegrees(differentialPivotDeg),
        Rotation2d.fromDegrees(differentialWristDeg));
  }

  public static Position fromRadians(
      double coralSlapdownRad,
      double algaeSlapdownRad,
      double elevatorMeters,
      double pivotRad,
      double differentialPivotRad,
      double differentialWristRad) {
    return of(
        new Rotation2d(coralSlapdownRad),
        new Rotation2d(algaeSlapdownRad),
        elevatorMeters,
        new Rotation2d(pivotRad),
        new Rotation2d(differentialPivotRad),
        new Rotation2d(differentialWristRad));
  }

  // Convenience/compat builders (legacy API) — maps old fields to reasonable defaults
  public static Position of(Rotation2d slapdown, Rotation2d wrist, double elevatorMeters) {
    return of(
        slapdown, // coralSlapdown
        slapdown, // algaeSlapdown (default to same as coral for legacy)
        elevatorMeters,
        new Rotation2d(), // pivotAngle = 0 deg
        new Rotation2d(), // differentialPivotAngle = 0 deg
        wrist); // differentialWristAngle
  }

  public static Position fromDegrees(double slapdownDeg, double wristDeg, double elevatorMeters) {
    return of(
        Rotation2d.fromDegrees(slapdownDeg), Rotation2d.fromDegrees(wristDeg), elevatorMeters);
  }

  public static Position fromRadians(double slapdownRad, double wristRad, double elevatorMeters) {
    return of(new Rotation2d(slapdownRad), new Rotation2d(wristRad), elevatorMeters);
  }

  // Withers
  public Position withCoralSlapdown(Rotation2d newCoralSlapdown) {
    return new Position(
        newCoralSlapdown,
        algaeSlapdownAngle,
        elevatorHeightMeters,
        pivotAngle,
        differentialPivotAngle,
        differentialWristAngle);
  }

  public Position withAlgaeSlapdown(Rotation2d newAlgaeSlapdown) {
    return new Position(
        coralSlapdownAngle,
        newAlgaeSlapdown,
        elevatorHeightMeters,
        pivotAngle,
        differentialPivotAngle,
        differentialWristAngle);
  }

  public Position withElevator(double newElevatorMeters) {
    return new Position(
        coralSlapdownAngle,
        algaeSlapdownAngle,
        newElevatorMeters,
        pivotAngle,
        differentialPivotAngle,
        differentialWristAngle);
  }

  public Position withPivot(Rotation2d newPivot) {
    return new Position(
        coralSlapdownAngle,
        algaeSlapdownAngle,
        elevatorHeightMeters,
        newPivot,
        differentialPivotAngle,
        differentialWristAngle);
  }

  public Position withDifferentialPivot(Rotation2d newDifferentialPivot) {
    return new Position(
        coralSlapdownAngle,
        algaeSlapdownAngle,
        elevatorHeightMeters,
        pivotAngle,
        newDifferentialPivot,
        differentialWristAngle);
  }

  public Position withDifferentialWrist(Rotation2d newDifferentialWrist) {
    return new Position(
        coralSlapdownAngle,
        algaeSlapdownAngle,
        elevatorHeightMeters,
        pivotAngle,
        differentialPivotAngle,
        newDifferentialWrist);
  }

  // Legacy/compat accessors
  public Rotation2d slapdownAngle() {
    // Prefer coral slapdown for legacy single-slapdown code paths
    return coralSlapdownAngle;
  }

  public Rotation2d wristAngle() {
    return differentialWristAngle;
  }

  public Position withSlapdown(Rotation2d newSlapdown) {
    return withCoralSlapdown(newSlapdown);
  }

  public Position withWrist(Rotation2d newWrist) {
    return withDifferentialWrist(newWrist);
  }

  public double elevatorHeightInInches() {
    return Units.metersToInches(elevatorHeightMeters);
  }

  @Override
  public String toString() {
    return "Position(coralSlapdownDeg=%.2f, algaeSlapdownDeg=%.2f, pivotDeg=%.2f, diffPivotDeg=%.2f, diffWristDeg=%.2f, elevator=%.3fm)"
        .formatted(
            coralSlapdownAngle.getDegrees(),
            algaeSlapdownAngle.getDegrees(),
            pivotAngle.getDegrees(),
            differentialPivotAngle.getDegrees(),
            differentialWristAngle.getDegrees(),
            elevatorHeightMeters);
  }
}
