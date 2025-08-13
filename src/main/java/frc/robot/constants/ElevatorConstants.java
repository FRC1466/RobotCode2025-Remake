// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public class ElevatorConstants {
  // Motor IDs
  public static final int masterMotorId = 17;
  public static final int followerMotorId = 16;

  // Current Limits
  public static final LoggedTunableNumber supplyCurrentLimit =
      new LoggedTunableNumber("Elevator/SupplyCurrentLimit", 60.0);
  public static final LoggedTunableNumber statorCurrentLimit =
      new LoggedTunableNumber("Elevator/StatorCurrentLimit", 120.0);

  // PID Constants
  public static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 50.0);
  public static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0.0);
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0.0);
  public static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.13);
  public static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.0);

  // Motion Magic Constants
  public static final LoggedTunableNumber accelerationConstraint =
      new LoggedTunableNumber("Elevator/AccelerationConstraint", 8);
  public static final LoggedTunableNumber accelerationConstraintAlgae =
      new LoggedTunableNumber("Elevator/AccelerationConstraintAlgae", 6);
  public static final LoggedTunableNumber accelerationConstraintDown =
      new LoggedTunableNumber("Elevator/AccelerationConstraintDown", 6);

  public static final LoggedTunableNumber velocityConstraint =
      new LoggedTunableNumber("Elevator/VelocityConstraint", 3);
  public static final LoggedTunableNumber velocityConstraintAlgae =
      new LoggedTunableNumber("Elevator/VelocityConstraintAlgae", 2.8);

  // Pulley specs
  public static final double pulleyTeeth = 24.0;
  public static final double beltPitchMm = 5.0; // HTD-5mm

  // Derived pitch radius in inches
  public static final double pulleyRadiusInches =
      ((pulleyTeeth * (beltPitchMm / 25.4)) / Math.PI) / 2.0; // (PD in inches) / 2

  // Gear ratio: motor revs per pulley rev
  public static final double elevatorGearRatio = 36.0 / 9.0; // 4.0

  public static final double elevatorRotationsToMeters(double rotations) {
    return (rotations / elevatorGearRatio)
        * (2 * Math.PI)
        * Units.inchesToMeters(pulleyRadiusInches);
  }

  public static final double elevatorMetersToRotations(double meters) {
    return (meters / ((2 * Math.PI) * Units.inchesToMeters(pulleyRadiusInches)))
        * elevatorGearRatio;
  }

  public static final double maxHeightMeters = 1.507;

  public static final LoggedTunableNumber stowed =
      new LoggedTunableNumber("Elevator/StowedPosition", elevatorRotationsToMeters(0.1));
}
