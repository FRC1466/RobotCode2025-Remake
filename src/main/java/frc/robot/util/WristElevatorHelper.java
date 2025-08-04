// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class WristElevatorHelper {
  private static RobotContainer robotContainer;

  private WristElevatorHelper() {}

  public static void init(RobotContainer RobotContainer) {
    robotContainer = RobotContainer;
  }

  public static Command setBothWantedState(Rotation2d wristAngle, double elevatorPositionMeters) {
    return new InstantCommand(
        () -> {
          robotContainer.getWrist().setWantedState(Wrist.WantedState.MOVE_TO_POSITION, wristAngle);
          robotContainer
              .getElevator()
              .setWantedState(Elevator.WantedState.MOVE_TO_POSITION, elevatorPositionMeters);
        });
  }
}
