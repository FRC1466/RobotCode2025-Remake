// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
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

  private final LoggedTunableNumber angleOffset =
      new LoggedTunableNumber("Superstructure/Visualizer/AngleOffset", 10);

  private final LoggedTunableNumber angleMultiplier =
      new LoggedTunableNumber("Superstructure/Visualizer/AngleMultiplier", -1);

  // Tunable numbers for 3D mechanism visualization
  private final LoggedTunableNumber comp1MaxHeight =
      new LoggedTunableNumber("Superstructure/Visualizer/Comp1MaxHeight", 0.925);
  private final LoggedTunableNumber comp2HeightMultiplier =
      new LoggedTunableNumber("Superstructure/Visualizer/Comp2HeightMultiplier", 0.5);
  private final LoggedTunableNumber comp3XOffset =
      new LoggedTunableNumber("Superstructure/Visualizer/Comp3XOffset", 0.28);
  private final LoggedTunableNumber comp3ZOffset =
      new LoggedTunableNumber("Superstructure/Visualizer/Comp3ZOffset", 0.365);
  private final LoggedTunableNumber comp3YOffset =
      new LoggedTunableNumber("Superstructure/Visualizer/Comp3YOffset", 0);
  private final LoggedTunableNumber comp3YRotOffset =
      new LoggedTunableNumber("Superstructure/Visualizer/Comp3YRotOffset", -28.934369);
  private final LoggedTunableNumber armAngleMultiplier =
      new LoggedTunableNumber("Superstructure/Visualizer/ArmAngleMultiplier", 1);

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
   * Updates both the elevator height and angle visualization, and logs 3D component poses.
   *
   * @param elevatorHeightMeters The height of the elevator in meters
   * @param angleRadians The angle in radians to visualize
   */
  public void update(double elevatorHeightMeters, double angleRadians) {
    update(elevatorHeightMeters);
    updateAngle(angleRadians);

    // 3D component visualization using tunable numbers
    double totalMaxHeight = comp1MaxHeight.getAsDouble() + comp2HeightMultiplier.getAsDouble();
    double heightRatio = elevatorHeightMeters / totalMaxHeight;

    double comp1Height =
        Math.min(heightRatio * comp1MaxHeight.getAsDouble(), comp1MaxHeight.getAsDouble());
    double comp2Height = comp1Height + (heightRatio * comp2HeightMultiplier.getAsDouble());

    double armAngleDegrees =
        armAngleMultiplier.getAsDouble() * Units.radiansToDegrees(angleRadians);

    Pose3d component1Pose = new Pose3d(new Translation3d(0, 0, comp1Height), new Rotation3d());
    Pose3d component2Pose = new Pose3d(new Translation3d(0, 0, comp2Height), new Rotation3d());
    Pose3d component3Pose =
        new Pose3d(
            new Translation3d(
                comp3XOffset.getAsDouble(),
                comp3YOffset.getAsDouble(),
                comp2Height + comp3ZOffset.getAsDouble()),
            new Rotation3d(
                0,
                Units.degreesToRadians(armAngleDegrees)
                    + Units.degreesToRadians(comp3YRotOffset.getAsDouble()),
                0));

    Pose3d[] componentPoses = new Pose3d[] {component1Pose, component2Pose, component3Pose};
    Logger.recordOutput("Mechanism3d/" + name + "/Components", componentPoses);
  }
}
