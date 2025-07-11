// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;

/**
 * As this does not use static variables, create an instance with a unique name to manage overrides
 * cro
 */
public class OverridePublisher implements BooleanSupplier {
  private final String name;

  /**
   * Creates a new OverridePublisher
   *
   * @param name The name of the override to publish to SmartDashboard. Should be unique
   */
  public OverridePublisher(String name) {
    this.name = "Overrides/" + name;
    SmartDashboard.putBoolean(this.name, false);
  }

  /**
   * Gets the current value of the override
   *
   * @return The boolean value of the override.
   */
  @Override
  public boolean getAsBoolean() {
    return SmartDashboard.getBoolean(name, false);
  }
}
