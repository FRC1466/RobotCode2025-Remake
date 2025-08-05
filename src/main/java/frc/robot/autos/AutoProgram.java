// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Function;

/** An autonomous program. */
public class AutoProgram {
  private final Auto auto;
  private final String label;

  private final Function<AutoFactory, Pair<Pose2d, Command>> commandFactory;

  /**
   * Create an autonomous program
   *
   * @param auto The {@link Auto} to associate with this program
   * @param label The human-readable label for the program
   * @param commandFactory The command factory for the program
   */
  public AutoProgram(
      final Auto auto,
      final String label,
      final Function<AutoFactory, Pair<Pose2d, Command>> commandFactory) {
    this.auto = auto;
    this.label = label;
    this.commandFactory = commandFactory;
  }

  /**
   * Get the {@link Auto} for this program
   *
   * @return The {@link Auto} for this program
   */
  Auto getAuto() {
    return auto;
  }

  /**
   * Get the label for this program
   *
   * @return The label for this program
   */
  public String getLabel() {
    return label;
  }

  /**
   * Construct the {@link Command} for this program from the provided {@link AutoFactory}
   *
   * @param autoFactory The {@link AutoFactory} to use when creating the {@link Command}
   * @return The {@link Command} for this program from the provided {@link AutoFactory}
   */
  public Command getCommand(final AutoFactory autoFactory) {
    return commandFactory.apply(autoFactory).getSecond();
  }

  public Pose2d getStartingPose(final AutoFactory autoFactory) {
    return commandFactory.apply(autoFactory).getFirst();
  }
}
