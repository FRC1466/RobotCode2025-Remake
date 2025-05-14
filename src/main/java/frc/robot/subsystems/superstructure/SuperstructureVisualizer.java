// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperstructureVisualizer {
  private final String name;
  private final LoggedMechanism2d mechanism =
      new LoggedMechanism2d(
          Units.inchesToMeters(28.0), Units.feetToMeters(7.0), new Color8Bit(Color.kDarkGray));
  private final LoggedMechanismLigament2d elevatorMechanism;

  private LoggedTunableNumber angleOffset =
      new LoggedTunableNumber("Superstructure/Visualizer/AngleOffset", 0.0);

  private LoggedTunableNumber angleMultiplier =
      new LoggedTunableNumber("Superstructure/Visualizer/AngleMultiplier", 1);

  // New angle visualization mechanism
  private final LoggedMechanism2d angleMechanism =
      new LoggedMechanism2d(1.0, 1.0, new Color8Bit(Color.kDarkGray));
  private final LoggedMechanismLigament2d angleLigament;

  public SuperstructureVisualizer(String name) {
    this.name = name;
    LoggedMechanismRoot2d root =
        mechanism.getRoot(
            name + " Root", superstructureOrigin2d.getX(), superstructureOrigin2d.getY());
    elevatorMechanism =
        root.append(
            new LoggedMechanismLigament2d(
                name + " Elevator",
                Units.inchesToMeters(26.0),
                elevatorAngle.getDegrees(),
                4.0,
                new Color8Bit(Color.kFirstBlue)));

    // Initialize the angle mechanism
    LoggedMechanismRoot2d angleRoot = angleMechanism.getRoot(name + " Angle Root", 0.5, 0.5);
    angleLigament =
        angleRoot.append(
            new LoggedMechanismLigament2d(
                name + " Angle",
                0.4, // Fixed length for visualization
                0.0, // Initial angle (will be updated)
                3.0, // Line width
                new Color8Bit(Color.kFirstRed)));
  }

  public void update(double elevatorHeightMeters) {
    if (Constants.getMode() != Mode.REAL) {
      elevatorMechanism.setLength(
          EqualsUtil.epsilonEquals(elevatorHeightMeters, 0.0)
              ? Units.inchesToMeters(1.0)
              : elevatorHeightMeters);
      Logger.recordOutput("Mechanism2d/" + name, mechanism);
    }
  }

  /**
   * Updates the angle visualization.
   *
   * @param angleRadians The angle in radians to visualize
   */
  public void updateAngle(double angleRadians) {
    if (Constants.getMode() != Mode.REAL) {
      // Convert radians to degrees for the mechanism visualization
      angleLigament.setAngle(
          (Units.radiansToDegrees(angleRadians) + angleOffset.getAsDouble())
              * Math.signum(angleMultiplier.getAsDouble()));
      Logger.recordOutput("Mechanism2d/" + name + " Angle", angleMechanism);
    }
  }

  /**
   * Updates both the elevator height and angle visualization.
   *
   * @param elevatorHeightMeters The height of the elevator in meters
   * @param angleRadians The angle in radians to visualize
   */
  public void update(double elevatorHeightMeters, double angleRadians) {
    update(elevatorHeightMeters);
    updateAngle(angleRadians);
  }
}
