// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import frc.robot.util.Position;

public class SuperstructurePositions {
  public static final Position STOW = Position.fromDegrees(0, 0, 0);
  public static final Position CORAL_STOW =
      Position.fromDegrees(0, 180, 0.82); // TODO: update values
  public static final Position ALGAE_STOW = Position.fromDegrees(0, 0, 0); // TODO: set values
  public static final Position CORAL_INTAKE = Position.fromDegrees(130, 180, 0.82);
  public static final Position CORAL_HANDOFF = Position.fromDegrees(0, 180, 0.82);
  // Reef levels (side 1) - HOLD and SCORE positions
  // Reef levels (LEFT side) - HOLD and SCORE positions
  public static final Position L1_LEFT_HOLD = Position.fromDegrees(0, 0, 0); // TODO: set values
  public static final Position L1_LEFT_SCORE = Position.fromDegrees(0, 0, 0); // TODO: set values
  public static final Position L2_LEFT_HOLD = Position.fromDegrees(0, 320, 0.3);
  public static final Position L2_LEFT_SCORE = Position.fromDegrees(0, 300, 0.3);
  public static final Position L3_LEFT_HOLD = Position.fromDegrees(0, 320, 0.7);
  public static final Position L3_LEFT_SCORE = Position.fromDegrees(0, 300, 0.7);
  public static final Position L4_LEFT_HOLD = Position.fromDegrees(0, 310, 1.45);
  public static final Position L4_LEFT_SCORE = Position.fromDegrees(0, 270, 1.45);

  // Reef levels (RIGHT side) - HOLD and SCORE positions
  public static final Position L1_RIGHT_HOLD = Position.fromDegrees(0, 0, 0); // TODO: set values
  public static final Position L1_RIGHT_SCORE = Position.fromDegrees(0, 0, 0); // TODO: set values
  public static final Position L2_RIGHT_HOLD = Position.fromDegrees(0, 40, 0.3);
  public static final Position L2_RIGHT_SCORE = Position.fromDegrees(0, 60, 0.3);
  public static final Position L3_RIGHT_HOLD = Position.fromDegrees(0, 40, 0.7);
  public static final Position L3_RIGHT_SCORE = Position.fromDegrees(0, 60, 0.7);
  public static final Position L4_RIGHT_HOLD = Position.fromDegrees(0, 50, 1.45);
  public static final Position L4_RIGHT_SCORE = Position.fromDegrees(0, 90, 1.45);

  // Algae / coral specific
  public static final Position ALGAE_INTAKE_L2 = Position.fromDegrees(0, 0, 0); // TODO: set values
  public static final Position ALGAE_INTAKE_L3 = Position.fromDegrees(0, 0, 0); // TODO: set values
  public static final Position PROCESSOR = Position.fromDegrees(0, 0, 0); // TODO: set values
  public static final Position BARGE = Position.fromDegrees(0, 0, 0); // TODO: set values
}
