// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;

public class IntakeCommands {
  public static Command intake(Superstructure superstructure) {
    return superstructure.runGoal(
        () ->
            superstructure.hasCoral()
                ? SuperstructureState.STOWREST
                : SuperstructureState.CORAL_INTAKE);
  }
}
