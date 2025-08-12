// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.SuperstructurePositions;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

public final class SuperstructurePositionChooser {
  private static final SendableChooser<Position> chooser = new SendableChooser<>();

  static {
    try {
      for (Field f : SuperstructurePositions.class.getDeclaredFields()) {
        if (Modifier.isPublic(f.getModifiers())
            && Modifier.isStatic(f.getModifiers())
            && f.getType() == Position.class) {

          Position pos = (Position) f.get(null);
          String name = f.getName();
          if ("STOW".equals(name)) {
            chooser.setDefaultOption(name, pos);
          } else {
            chooser.addOption(name, pos);
          }
        }
      }
    } catch (IllegalAccessException e) {
      System.err.println("Failed to build SuperstructurePosition chooser: " + e.getMessage());
    }
    SmartDashboard.putData("Superstructure Position", chooser);
  }

  private SuperstructurePositionChooser() {}

  public static Position getSelected() {
    Position p = chooser.getSelected();
    return p != null ? p : SuperstructurePositions.STOW;
  }
}
