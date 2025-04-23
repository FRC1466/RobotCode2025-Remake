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
import java.util.Map;

public class PathfindConstants {
  public static final Pose2d[][] blueTargetPoseReef = createReefPoses();
  public static final Pose2d[][] blueTargetPoseReefApproach = createReefApproachPoses();

  public static final Pose2d[][] redTargetPoseReef = flipPoseArray(blueTargetPoseReef);
  public static final Pose2d[][] redTargetPoseReefApproach =
      flipPoseArray(blueTargetPoseReefApproach);

  private static Pose2d[][] createReefPoses() {
    Pose2d[][] reefPoses = new Pose2d[6][2]; // 6 tags, left and right branches for each

    for (int i = 0; i < 6; i++) {
      // Calculate correct branch indices - adjust by subtracting 1 to fix the "one branch to the
      // right" issue
      // Wrap around using modulo to handle the case where i=0 becomes -1
      int tagOffset = 1; // This offset fixes the "one to the right" issue
      int adjustedFaceIndex =
          (i - tagOffset + 6) % 6; // Add 6 before modulo to avoid negative indices

      int rightBranchIndex = (adjustedFaceIndex * 2) % 12; // Even indices (0,2,4,6,8,10)
      int leftBranchIndex = (adjustedFaceIndex * 2 + 1) % 12; // Odd indices (1,3,5,7,9,11)

      // Get the left branch
      Map<FieldConstants.ReefLevel, Pose2d> leftBranchMap =
          FieldConstants.Reef.branchPositions2d.get(leftBranchIndex);

      // Get the right branch
      Map<FieldConstants.ReefLevel, Pose2d> rightBranchMap =
          FieldConstants.Reef.branchPositions2d.get(rightBranchIndex);

      // Calculate left branch pose
      Pose2d leftCenterFace = leftBranchMap.get(FieldConstants.ReefLevel.L2);
      double approachDistance = 0.575; // Distance from the reef face

      // Create approach pose by moving AWAY from the branch and rotating 180 degrees
      reefPoses[i][0] =
          new Pose2d(
              leftCenterFace.getX()
                  + (approachDistance * Math.cos(leftCenterFace.getRotation().getRadians())),
              leftCenterFace.getY()
                  + (approachDistance * Math.sin(leftCenterFace.getRotation().getRadians())),
              leftCenterFace.getRotation().rotateBy(Rotation2d.fromDegrees(180)));

      // Calculate right branch pose
      Pose2d rightCenterFace = rightBranchMap.get(FieldConstants.ReefLevel.L2);

      // Create approach pose by moving AWAY from the branch and rotating 180 degrees
      reefPoses[i][1] =
          new Pose2d(
              rightCenterFace.getX()
                  + (approachDistance * Math.cos(rightCenterFace.getRotation().getRadians())),
              rightCenterFace.getY()
                  + (approachDistance * Math.sin(rightCenterFace.getRotation().getRadians())),
              rightCenterFace.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    }

    return reefPoses;
  }

  private static Pose2d[][] createReefApproachPoses() {
    Pose2d[][] reefPoses = new Pose2d[6][2]; // 6 tags, left and right branches for each

    for (int i = 0; i < 6; i++) {
      // Calculate correct branch indices - adjust by subtracting 1 to fix the "one branch to the
      // right" issue
      // Wrap around using modulo to handle the case where i=0 becomes -1
      int tagOffset = 1; // This offset fixes the "one to the right" issue
      int adjustedFaceIndex =
          (i - tagOffset + 6) % 6; // Add 6 before modulo to avoid negative indices

      int rightBranchIndex = (adjustedFaceIndex * 2) % 12; // Even indices (0,2,4,6,8,10)
      int leftBranchIndex = (adjustedFaceIndex * 2 + 1) % 12; // Odd indices (1,3,5,7,9,11)

      // Get the left branch
      Map<FieldConstants.ReefLevel, Pose2d> leftBranchMap =
          FieldConstants.Reef.branchPositions2d.get(leftBranchIndex);

      // Get the right branch
      Map<FieldConstants.ReefLevel, Pose2d> rightBranchMap =
          FieldConstants.Reef.branchPositions2d.get(rightBranchIndex);

      // Calculate left branch pose
      Pose2d leftCenterFace = leftBranchMap.get(FieldConstants.ReefLevel.L2);
      double approachDistance = 1.475; // Distance from the reef face

      // Create approach pose by moving AWAY from the branch and rotating 180 degrees
      reefPoses[i][0] =
          new Pose2d(
              leftCenterFace.getX()
                  + (approachDistance * Math.cos(leftCenterFace.getRotation().getRadians())),
              leftCenterFace.getY()
                  + (approachDistance * Math.sin(leftCenterFace.getRotation().getRadians())),
              leftCenterFace.getRotation().rotateBy(Rotation2d.fromDegrees(180)));

      // Calculate right branch pose
      Pose2d rightCenterFace = rightBranchMap.get(FieldConstants.ReefLevel.L2);

      // Create approach pose by moving AWAY from the branch and rotating 180 degrees
      reefPoses[i][1] =
          new Pose2d(
              rightCenterFace.getX()
                  + (approachDistance * Math.cos(rightCenterFace.getRotation().getRadians())),
              rightCenterFace.getY()
                  + (approachDistance * Math.sin(rightCenterFace.getRotation().getRadians())),
              rightCenterFace.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    }

    return reefPoses;
  }

  // Red Target Poses for Station
  public static final Pose2d[] redTargetPoseStation = {
    new Pose2d(16.195, 7.19, Rotation2d.fromDegrees(-125.000)),
    new Pose2d(16.288, 0.937, Rotation2d.fromDegrees(125.000)),
  };

  public static final Pose2d redTargetPoseProcessor =
      new Pose2d(11.496, 7.495, Rotation2d.fromDegrees(90.000));

  public static final double blueTargetPoseXBarge = 8;
}
