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
import frc.robot.constants.FieldConstants.CoralObjective;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;

public class MirrorUtil {
  @Setter @Getter private static BooleanSupplier mirror;

  public static CoralObjective apply(CoralObjective coralObjective) {
    if (!mirror.getAsBoolean()) return coralObjective;
    int shiftedBranchId = coralObjective.branchId() - 1;
    if (shiftedBranchId == -1) {
      shiftedBranchId = 11;
    }
    int flippedBranchId = 11 - shiftedBranchId;
    flippedBranchId = (++flippedBranchId == 12) ? 0 : flippedBranchId;
    return new CoralObjective(flippedBranchId, coralObjective.reefLevel());
  }

  public static Pose2d apply(Pose2d pose) {
    if (!mirror.getAsBoolean()) return pose;
    return new Pose2d(
        pose.getX(),
        FieldConstants.fieldWidth - pose.getY(),
        new Rotation2d(pose.getRotation().getCos(), -pose.getRotation().getSin()));
  }
}
