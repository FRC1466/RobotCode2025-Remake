// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

public class IntakeConstants {
  // Coral operations. Because this is a theoretical voltage, voltage is purely positive or
  // negative. Positive is intake, negative is outtake.
  public static final RollerVoltages coralIntake =
      new RollerVoltages(1, 1); // Pick coral off ground : slapdown
  public static final RollerVoltages coralGrip =
      new RollerVoltages(1, 1); // Hold coral : either mechanism
  public static final RollerVoltages coralHandoff =
      new RollerVoltages(1, -1); // Coral transfer : slapdown -> claw
  public static final RollerVoltages coralOuttake =
      new RollerVoltages(-1, -1); // Eject coral : either mechanism

  // Algae operations
  public static final RollerVoltages algaeIntake = new RollerVoltages(1, 0); // Claw
  public static final RollerVoltages algaeHold = new RollerVoltages(1, 0); // Claw
  public static final RollerVoltages algaeEject = new RollerVoltages(-1, 0); // Claw

  public static final double currentThresholdAlgaeIntake = 10.0;
  public static final double currentThresholdAlgaeHold = -80.0;

  public static final double distanceThresholdCoralIntake = 0.06;

  public static final class RollerVoltages {
    public final double clawVoltage;
    public final double slapdownVoltage;

    public RollerVoltages(double clawVoltage, double slapdownVoltage) {
      this.clawVoltage = clawVoltage;
      this.slapdownVoltage = slapdownVoltage;
    }
  }
}
