// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.archive.superstructure;

import static frc.robot.subsystems.archive.superstructure.SuperstructureConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.EqualsUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperstructureVisualizer {
  private static final double minElevatorVizLengthMeters = Units.inchesToMeters(1.0);
  private static final double elevatorMechanismWidthMeters = Units.inchesToMeters(28.0);
  private static final double elevatorMechanismHeightMeters = Units.feetToMeters(7.0);
  private static final double elevatorLigamentDefaultLengthMeters = Units.inchesToMeters(26.0);
  private static final double elevatorLigamentLineWidth = 4.0;
  private static final double angleMechanismSize = 1.0;
  private static final double angleLigamentLength = 0.4;
  private static final double angleLigamentLineWidth = 3.0;

  private final String name;
  private final LoggedMechanism2d elevatorMechanism2d;
  private final LoggedMechanismLigament2d elevatorLigament;
  private final LoggedMechanism2d angleMechanism2d;
  private final LoggedMechanismLigament2d angleLigament;

  public SuperstructureVisualizer(String name) {
    this.name = name;

    this.elevatorMechanism2d =
        new LoggedMechanism2d(
            elevatorMechanismWidthMeters,
            elevatorMechanismHeightMeters,
            new Color8Bit(Color.kDarkGray));
    LoggedMechanismRoot2d elevatorRoot =
        elevatorMechanism2d.getRoot(
            name + "Root", superstructureOrigin2d.getX(), superstructureOrigin2d.getY());
    this.elevatorLigament =
        elevatorRoot.append(
            new LoggedMechanismLigament2d(
                name + "Elevator",
                elevatorLigamentDefaultLengthMeters,
                elevatorAngle.getDegrees(),
                elevatorLigamentLineWidth,
                new Color8Bit(Color.kFirstBlue)));

    this.angleMechanism2d =
        new LoggedMechanism2d(
            angleMechanismSize, angleMechanismSize, new Color8Bit(Color.kDarkGray));
    LoggedMechanismRoot2d angleRoot =
        angleMechanism2d.getRoot(
            name + "AngleRoot", angleMechanismSize / 2, angleMechanismSize / 2);
    this.angleLigament =
        angleRoot.append(
            new LoggedMechanismLigament2d(
                name + "Angle",
                angleLigamentLength,
                0.0,
                angleLigamentLineWidth,
                new Color8Bit(Color.kFirstRed)));
  }

  public void update(
      double elevatorHeightMeters,
      double armAngleRadians,
      boolean hasCoral,
      boolean hasAlgae,
      Pose2d robotPose) {
    if (Constants.getMode() != Mode.REAL) {
      updateElevator2dVisuals(elevatorHeightMeters);
      updateAngle2dVisuals(armAngleRadians);
    }
    update3dVisuals(elevatorHeightMeters, armAngleRadians, hasAlgae, hasCoral, robotPose);
  }

  private void updateElevator2dVisuals(double elevatorHeightMeters) {
    elevatorLigament.setLength(
        EqualsUtil.epsilonEquals(elevatorHeightMeters, 0.0)
            ? minElevatorVizLengthMeters
            : elevatorHeightMeters);
    Logger.recordOutput("Mechanism2d/" + name + "Elevator", elevatorMechanism2d);
  }

  private void updateAngle2dVisuals(double armAngleRadians) {
    double angleDegrees = Units.radiansToDegrees(armAngleRadians);
    angleLigament.setAngle((angleDegrees + 10.0) * -1.0);
    Logger.recordOutput("Mechanism2d/" + name + "Angle", angleMechanism2d);
  }

  private void update3dVisuals(
      double elevatorHeightMeters,
      double armAngleRadians,
      boolean hasAlgae,
      boolean hasCoral,
      Pose2d robotPose) {
    double totalVisualMaxHeight = 0.925 + 0.5;
    double heightRatio =
        totalVisualMaxHeight == 0 ? 0 : elevatorHeightMeters / totalVisualMaxHeight;

    double comp1VisualHeight = Math.min(heightRatio * 0.925, 0.925);
    double comp2VisualHeight = comp1VisualHeight + (heightRatio * 0.5);

    double armVisualAngleDegrees = 1.0 * Units.radiansToDegrees(armAngleRadians);

    Pose3d component1Pose =
        new Pose3d(new Translation3d(0, 0, comp1VisualHeight), new Rotation3d());
    Pose3d component2Pose =
        new Pose3d(new Translation3d(0, 0, comp2VisualHeight), new Rotation3d());
    Pose3d component3Pose =
        new Pose3d(
            new Translation3d(0.28, 0.0, comp2VisualHeight + 0.365),
            new Rotation3d(0, Units.degreesToRadians(armVisualAngleDegrees + -28.934369), 0));

    Logger.recordOutput(
        "Mechanism3d/" + name + "/Components", component1Pose, component2Pose, component3Pose);

    // Handle algae visualization
    if (hasAlgae) {
      Logger.recordOutput(
          "Mechanism3d/" + name + "/Algae",
          new Translation3d[] {
            new Pose3d(robotPose)
                .transformBy(new Transform3d(Pose3d.kZero, component3Pose))
                .transformBy(
                    new Transform3d(
                        -0.22,
                        0.0,
                        0.13,
                        new Rotation3d(
                            Units.degreesToRadians(0.0),
                            Units.degreesToRadians(0.0),
                            Units.degreesToRadians(0.0))))
                .getTranslation()
          });
    } else {
      Logger.recordOutput("Mechanism3d/" + name + "/Algae", new Translation3d[] {});
    }

    // Handle coral visualization
    if (hasCoral) {
      Logger.recordOutput(
          "Mechanism3d/" + name + "/Coral",
          new Pose3d[] {
            new Pose3d(robotPose)
                .transformBy(new Transform3d(Pose3d.kZero, component3Pose))
                .transformBy(
                    new Transform3d(
                        0.0,
                        0.0,
                        0.3,
                        new Rotation3d(
                            Units.degreesToRadians(0.0),
                            Units.degreesToRadians(40.107),
                            Units.degreesToRadians(0.0))))
          });
    } else {
      Logger.recordOutput("Mechanism3d/" + name + "/Coral", new Pose3d[] {});
    }
  }
}
