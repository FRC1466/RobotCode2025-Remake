// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.util.FlipField.flipPoseArray;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.LoggedTunableNumber;
import java.util.Map;

public class PathfindConstants {
  private static final LoggedTunableNumber reefFaceDistance =
      new LoggedTunableNumber("Pathfind/ReefFaceDistance", 0.575);
  private static final LoggedTunableNumber reefApproachDistance =
      new LoggedTunableNumber("Pathfind/ReefApproachDistance", 1.2);

  public static Pose2d[][] getTargetPoseReef() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? getBlueTargetPoseReef()
        : getRedTargetPoseReef();
  }

  public static Pose2d[][] getTargetPoseReefApproach() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? getBlueTargetPoseReefApproach()
        : getRedTargetPoseReefApproach();
  }

  public static Pose2d[][] getBlueTargetPoseReef() {
    return computeReefPoses(reefFaceDistance.get());
  }

  public static Pose2d[][] getBlueTargetPoseReefApproach() {
    return computeReefPoses(reefApproachDistance.get());
  }

  public static Pose2d[][] getRedTargetPoseReef() {
    return flipPoseArray(getBlueTargetPoseReef());
  }

  public static Pose2d[][] getRedTargetPoseReefApproach() {
    return flipPoseArray(getBlueTargetPoseReefApproach());
  }

  private static Pose2d[][] computeReefPoses(double approachDistance) {
    Pose2d[][] reefPoses = new Pose2d[6][2];
    for (int i = 0; i < 6; i++) {
      int adjustedFaceIndex = (i - 1 + 6) % 6;
      int rightBranchIndex = (adjustedFaceIndex * 2) % 12;
      int leftBranchIndex = (adjustedFaceIndex * 2 + 1) % 12;

      Map<FieldConstants.ReefLevel, Pose2d> leftMap =
          FieldConstants.Reef.branchPositions2d.get(leftBranchIndex);
      Map<FieldConstants.ReefLevel, Pose2d> rightMap =
          FieldConstants.Reef.branchPositions2d.get(rightBranchIndex);

      Pose2d leftCenter = leftMap.get(FieldConstants.ReefLevel.L2);
      Pose2d rightCenter = rightMap.get(FieldConstants.ReefLevel.L2);

      // move away from the face and rotate 180°
      reefPoses[i][0] =
          new Pose2d(
              leftCenter.getX()
                  + approachDistance * Math.cos(leftCenter.getRotation().getRadians()),
              leftCenter.getY()
                  + approachDistance * Math.sin(leftCenter.getRotation().getRadians()),
              leftCenter.getRotation().rotateBy(Rotation2d.fromDegrees(180)));

      reefPoses[i][1] =
          new Pose2d(
              rightCenter.getX()
                  + approachDistance * Math.cos(rightCenter.getRotation().getRadians()),
              rightCenter.getY()
                  + approachDistance * Math.sin(rightCenter.getRotation().getRadians()),
              rightCenter.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    }
    return reefPoses;
  }

  public static final Pose2d[] redTargetPoseStation = {
    new Pose2d(16.195, 7.19, Rotation2d.fromDegrees(-125.0)),
    new Pose2d(16.288, 0.937, Rotation2d.fromDegrees(125.0)),
  };
  public static final Pose2d redTargetPoseProcessor =
      new Pose2d(11.496, 7.495, Rotation2d.fromDegrees(90.0));
  public static final double blueTargetPoseXBarge = 8.0;
}
