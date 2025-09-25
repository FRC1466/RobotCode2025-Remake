// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

public class IntakeConstants {
  // Coral operations
  public static final RollerVoltages coralIntake = new RollerVoltages(-1.25, -4);
  public static final RollerVoltages coralGrip = new RollerVoltages(0.2, 0);
  public static final RollerVoltages coralOuttake = new RollerVoltages(-10, 0);
  public static final RollerVoltages coralOuttakeL1 = new RollerVoltages(4, 0);
  public static final RollerVoltages coralBackup = new RollerVoltages(0.5, 0);

  // Algae operations
  public static final RollerVoltages algaeIntake = new RollerVoltages(3.0, 0);
  public static final RollerVoltages algaeHold = new RollerVoltages(0.4, 0);
  public static final RollerVoltages algaeEject = new RollerVoltages(-2.5, 0);

  public static final double currentThresholdAlgaeIntake = 10.0;
  public static final double currentThresholdAlgaeHold = -80.0;

  public static final double distanceThresholdCoralIntake = 0.06;

  public static final class RollerVoltages {
    public final double endEffectorVoltage;
    public final double starWheelVoltage;

    public RollerVoltages(double endEffectorVoltage, double starWheelVoltage) {
      this.endEffectorVoltage = endEffectorVoltage;
      this.starWheelVoltage = starWheelVoltage;
    }
  }
}
