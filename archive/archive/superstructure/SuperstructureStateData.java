// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.archive.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.archive.superstructure.manipulator.Manipulator;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Builder(toBuilder = true, access = AccessLevel.PACKAGE)
@Getter
public class SuperstructureStateData {
  @Builder.Default
  private final SuperstructurePose pose = new SuperstructurePose(() -> 0.0, () -> Rotation2d.kZero);

  @Builder.Default private final Manipulator.MailboxGoal MailboxGoal = Manipulator.MailboxGoal.IDLE;
  @Builder.Default private final Height height = Height.BOTTOM;

  /** What height is the carriage above? */
  @RequiredArgsConstructor
  @Getter
  public enum Height {
    BOTTOM(0),
    FIRST_STAGE(SuperstructureConstants.stage1ToStage2Height),
    SECOND_STAGE(SuperstructureConstants.stage2ToStage3Height);

    private final double position;

    public boolean lowerThan(Height other) {
      return position <= other.position;
    }

    public boolean upperThan(Height other) {
      return position > other.position;
    }
  }
}
