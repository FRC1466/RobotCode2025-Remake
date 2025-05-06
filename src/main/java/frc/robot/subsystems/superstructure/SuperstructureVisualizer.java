// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.math.geometry.*;
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
  private final String name;
  private final LoggedMechanism2d mechanism =
      new LoggedMechanism2d(
          Units.inchesToMeters(28.0), Units.feetToMeters(7.0), new Color8Bit(Color.kDarkGray));
  private final LoggedMechanismLigament2d elevatorMechanism;

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
}
