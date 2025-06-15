// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;

public class IntakeCommands {
  public static Command intake(Superstructure superstructure) {
    return superstructure
        .runGoal(
            () -> {
              if (superstructure.hasCoral()) {
                return SuperstructureState.STOWREST;
              } else {
                return SuperstructureState.CORAL_INTAKE;
              }
            })
        .alongWith(
            Commands.waitUntil(superstructure::hasCoral)
                .andThen(
                    RobotContainer.controllerRumbleCommand()
                        .withTimeout(0.1)
                        .andThen(Commands.waitSeconds(0.1))
                        .repeatedly()));
  }
}
