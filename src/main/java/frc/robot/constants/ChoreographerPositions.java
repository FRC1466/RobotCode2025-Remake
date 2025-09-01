// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;

import frc.robot.util.Position;

public class ChoreographerPositions {
  public static final Position STOW = Position.fromDegrees(0, 0, 0, 10, 0, 0);
  public static final Position PRE_MATCH = Position.fromDegrees(0, 27.5, 0, 57.5, 55, 0);
  public static final Position CORAL_STOW = Position.fromDegrees(0, 0, 0, 30, 30, 0);
  public static final Position CORAL_STOW_L1 = Position.fromDegrees(75, 0, 0, 30, 120, 90);
  public static final Position ALGAE_STOW = Position.fromDegrees(0, 0, 0, 20, 20, 0);
  public static final Position ALGAE_STOW_SAFE = Position.fromDegrees(0, 180, 0, 20, 20, 0);
  public static final Position CORAL_HANDOFF = Position.fromDegrees(75, 0, 0, 35, 120, 90);
  public static final Position CORAL_INTAKE = CORAL_HANDOFF.withCoralSlapdown(fromDegrees(225));

  // General scoring position(s)
  public static final Position L1 = Position.fromDegrees(120, 70, 0, 30, 30, 0);
  public static final Position L1_FINAL = Position.fromDegrees(120, 110, 0, 30, 30, 0);

  // Reef levels (FRONT side) - SCORE positions
  public static final Position L2_FRONT_SCORE = Position.fromDegrees(0, 0, 0, 37.5, -20, 0);
  public static final Position L3_FRONT_SCORE = Position.fromDegrees(0, 0, 0.35, 37.5, -20, 0);
  public static final Position L4_FRONT_SCORE = Position.fromDegrees(0, 0, 1.2, 17.5, 65, 0);

  // Reef levels (BACK side) - SCORE positions
  public static final Position L2_BACK_SCORE = Position.fromDegrees(0, 0, 0, -10, -40, 0);
  public static final Position L3_BACK_SCORE = Position.fromDegrees(0, 0, 0.3, -10, -20, 0);
  public static final Position L4_BACK_SCORE = Position.fromDegrees(0, 0, 1.2, -2.5, -88, 0);

  // Algae specific (FRONT side)
  public static final Position ALGAE_GROUND_INTAKE = Position.fromDegrees(0, 140, 0, 60, 80, 0);
  public static final Position ALGAE_GROUND_INTAKE_TRANSITIONING =
      Position.fromDegrees(0, 180, 0, 60, 80, 0);
  public static final Position ALGAE_FRONT_INTAKE_L2 = Position.fromDegrees(0, 0, .35, 40, 50, 0);
  public static final Position ALGAE_FRONT_INTAKE_L3 = Position.fromDegrees(0, 0, 0.6, 30, 60, 0);
  public static final Position ALGAE_FRONT_PROCESSOR = Position.fromDegrees(0, 0, 0, 90, 10, 0);
  public static final Position ALGAE_FRONT_BARGE = Position.fromDegrees(0, 0, 1.2, 20, -10, 0);

  // Algae specific (BACK side)
  public static final Position ALGAE_BACK_INTAKE_L2 =
      Position.fromDegrees(0, 0, 0.1, -10, -77.5, 0);
  public static final Position ALGAE_BACK_INTAKE_L3 = Position.fromDegrees(0, 0, .475, -5, -80, 0);
  public static final Position ALGAE_BACK_BARGE = Position.fromDegrees(0, 0, 0, 0, 0, 0);
}
