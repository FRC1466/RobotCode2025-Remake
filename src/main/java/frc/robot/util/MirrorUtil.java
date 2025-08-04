// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.FieldConstants;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;

public class MirrorUtil {
  @Setter @Getter private static BooleanSupplier mirror;

  public static Pose2d apply(Pose2d pose) {
    if (!mirror.getAsBoolean()) return pose;
    return new Pose2d(
        pose.getX(),
        FieldConstants.FIELD_HEIGHT - pose.getY(),
        new Rotation2d(pose.getRotation().getCos(), -pose.getRotation().getSin()));
  }
}
