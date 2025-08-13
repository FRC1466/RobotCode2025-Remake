// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ReefConstants;
import frc.robot.constants.SuperstructureConstants;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  @AutoLogOutput private Pose2d robotToFieldFromSwerveDriveOdometry = new Pose2d();
  private ChassisSpeeds robotChassisSpeeds = new ChassisSpeeds();

  public record SwerveDriveObservation(Pose2d robotPose, ChassisSpeeds robotSpeeds) {}

  public void addPoseObservation(SwerveDriveObservation observation) {
    this.robotToFieldFromSwerveDriveOdometry = observation.robotPose;
    this.robotChassisSpeeds = observation.robotSpeeds;
  }

  public Pose2d getRobotPoseFromSwerveDriveOdometry() {
    return robotToFieldFromSwerveDriveOdometry;
  }

  public ChassisSpeeds getRobotChassisSpeeds() {
    return robotChassisSpeeds;
  }

  public ReefConstants.ScoringCoralMappingRotationToTagID
      getValidTagIDsFromClosest60DegreeRotation() {
    return getValidTagIDsFromClosest60DegreeRotation(getClosest60Degrees());
  }

  public ReefConstants.ScoringCoralMappingRotationToTagID getValidTagIDsFromClosest60DegreeRotation(
      Rotation2d closest60Degrees) {
    var ids =
        FieldConstants.isBlueAlliance()
            ? ReefConstants.blueAllianceAngleToTagIdsMap.get(closest60Degrees)
            : ReefConstants.redAllianceAngleToTagIdsMap.get(closest60Degrees);
    return ids;
  }

  public Rotation2d getClosest60Degrees() {
    double[] list = {60, 120, 180, -60, -120, 0};
    double desiredRotation = 0;
    for (double e : list) {
      var rotation = Rotation2d.fromDegrees(e);
      if (robotToFieldFromSwerveDriveOdometry.getRotation().minus(rotation).getDegrees() < 30.0
          && robotToFieldFromSwerveDriveOdometry.getRotation().minus(rotation).getDegrees()
              >= -30) {
        desiredRotation = e;
      }
    }
    Logger.recordOutput("RobotState/Closest60DegreeAngle", desiredRotation);
    return Rotation2d.fromDegrees(desiredRotation);
  }

  public Rotation2d getClosestRotationToFaceNearestReefFace() {
    int correctTagID = getClosestTagId();
    Logger.recordOutput("ClosestTagId", correctTagID);

    var mapToUse =
        FieldConstants.isBlueAlliance()
            ? ReefConstants.blueAllianceAngleToTagIdsMap
            : ReefConstants.redAllianceAngleToTagIdsMap;
    Rotation2d frontScoreRotation = new Rotation2d();

    for (Map.Entry<Rotation2d, ReefConstants.ScoringCoralMappingRotationToTagID> entry :
        mapToUse.entrySet()) {
      if (entry.getValue().frontId == correctTagID) {
        frontScoreRotation = entry.getKey();
      }
    }
    return frontScoreRotation;
  }

  public int getClosestTagId() {
    var pose = RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry();
    List<Pose2d> possiblePoses = List.of();
    int correctTagID = 0;

    if (FieldConstants.isBlueAlliance()) {
      possiblePoses = ReefConstants.blueAlliancePoseToTagIdsMap.keySet().stream().toList();
      correctTagID = ReefConstants.blueAlliancePoseToTagIdsMap.get(pose.nearest(possiblePoses));

    } else {
      possiblePoses = ReefConstants.redAlliancePoseToTagIdsMap.keySet().stream().toList();
      correctTagID = ReefConstants.redAlliancePoseToTagIdsMap.get(pose.nearest(possiblePoses));
    }

    return correctTagID;
  }

  public SuperstructureConstants.ScoringDirection getFacingSideRelativeToClosestTag() {
    int tagId = getClosestTagId();
    Pose2d tagPose = FieldConstants.getTagPose(tagId).toPose2d();
    Pose2d robotPose = getRobotPoseFromSwerveDriveOdometry();

    // Positive means tag rotation is CCW from robot rotation
    double difference = tagPose.getRotation().minus(robotPose.getRotation()).getDegrees();
    // Normalize to [-180,180)
    difference = ((difference + 180) % 360 + 360) % 360 - 180;
    Logger.recordOutput("RobotState/TagRotationDelta", difference);

    // Decide shortest turn direction (LEFT = counter-clockwise, RIGHT = clockwise)
    if (Math.abs(difference) < 1.0) {
      // Already effectively aligned; pick an arbitrary stable direction (adjust if you add an
      // ALIGNED state)
      return SuperstructureConstants.ScoringDirection.LEFT;
    }
    return difference > 0
        ? SuperstructureConstants.ScoringDirection.LEFT
        : SuperstructureConstants.ScoringDirection.RIGHT;
  }
}
