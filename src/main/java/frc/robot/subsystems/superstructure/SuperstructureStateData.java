// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.superstructure.manipulator.Manipulator;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;

@Builder(toBuilder = true, access = AccessLevel.PACKAGE)
@Getter
public class SuperstructureStateData {
  @Builder.Default
  private final SuperstructurePose pose = new SuperstructurePose(() -> 0.0, () -> Rotation2d.kZero);

  @Builder.Default private final Manipulator.MailboxGoal MailboxGoal = Manipulator.MailboxGoal.IDLE;
}
