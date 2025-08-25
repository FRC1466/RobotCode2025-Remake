// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.ChoreographerConstants.ScoringDirection;

@SuppressWarnings("UnusedVariable")
public class FieldConstants {
  public static final AprilTagFieldLayout FIELD_LAYOUT;

  static {
    FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
  }

  public static final double FIELD_HEIGHT = 8.0518;
  public static final double FIELD_LENGTH = 17.548249;

  // April tag IDs
  public static final int RED_LEFT_CORAL_STATION = 1;
  public static final int RED_RIGHT_CORAL_STATION = 2;
  public static final int RED_PROCESSOR = 3;
  public static final int RED_RIGHT_NET = 4;
  public static final int RED_LEFT_NET = 5;
  public static final int RED_REEF_LEFT_DRIVER_STATION = 6;
  public static final int RED_REEF_CENTER_DRIVER_STATION = 7;
  public static final int RED_REEF_RIGHT_DRIVER_STATION = 8;
  public static final int RED_REEF_RIGHT_BARGE = 9;
  public static final int RED_REEF_CENTER_BARGE = 10;
  public static final int RED_REEF_LEFT_BARGE = 11;
  public static final int BLUE_RIGHT_CORAL_STATION = 12;
  public static final int BLUE_LEFT_CORAL_STATION = 13;
  public static final int BLUE_LEFT_BARGE = 14;
  public static final int BLUE_RIGHT_BARGE = 15;
  public static final int BLUE_PROCESSOR = 16;
  public static final int BLUE_REEF_RIGHT_DRIVER_STATION = 17;
  public static final int BLUE_REEF_CENTER_DRIVER_STATION = 18;
  public static final int BLUE_REEF_LEFT_DRIVER_STATION = 19;
  public static final int BLUE_REEF_RIGHT_BARGE = 20;
  public static final int BLUE_REEF_CENTER_BARGE = 21;
  public static final int BLUE_REEF_LEFT_BARGE = 22;

  public static final Translation2d RED_MARK_1 = new Translation2d(16.329, 2.197);
  public static final Translation2d RED_MARK_2 = new Translation2d(16.329, 4.026);
  public static final Translation2d RED_MARK_3 = new Translation2d(16.329, 5.855);
  public static final Translation2d BLUE_MARK_1 = new Translation2d(1.219, 5.855);
  public static final Translation2d BLUE_MARK_2 = new Translation2d(1.219, 4.026);
  public static final Translation2d BLUE_MARK_3 = new Translation2d(1.219, 2.197);

  public static final Pose2d FAR_LEFT_STARTING_POSE_BLUE = new Pose2d(7.2, 7, Rotation2d.k180deg);
  public static final Pose2d FAR_LEFT_STARTING_POSE_RED =
      new Pose2d(
          FIELD_LENGTH - FAR_LEFT_STARTING_POSE_BLUE.getX(),
          FIELD_HEIGHT - FAR_LEFT_STARTING_POSE_BLUE.getY(),
          Rotation2d.kZero);
  public static final Pose2d FAR_RIGHT_STARTING_POSE_BLUE =
      new Pose2d(
          FAR_LEFT_STARTING_POSE_BLUE.getX(),
          FIELD_HEIGHT - FAR_LEFT_STARTING_POSE_BLUE.getY(),
          Rotation2d.k180deg);
  public static final Pose2d FAR_RIGHT_STARTING_POSE_RED =
      new Pose2d(
          FIELD_LENGTH - FAR_RIGHT_STARTING_POSE_BLUE.getX(),
          FIELD_HEIGHT - FAR_RIGHT_STARTING_POSE_BLUE.getY(),
          Rotation2d.kZero);

  public static final Pose2d LEFT_STARTING_POSE_BLUE = new Pose2d(7.2, 6.14, Rotation2d.k180deg);
  public static final Pose2d LEFT_STARTING_POSE_RED =
      new Pose2d(
          FIELD_LENGTH - LEFT_STARTING_POSE_BLUE.getX(),
          FIELD_HEIGHT - LEFT_STARTING_POSE_BLUE.getY(),
          Rotation2d.kZero);
  public static final Pose2d RIGHT_STARTING_POSE_BLUE =
      new Pose2d(
          LEFT_STARTING_POSE_BLUE.getX(),
          FIELD_HEIGHT - LEFT_STARTING_POSE_BLUE.getY(),
          Rotation2d.k180deg);
  public static final Pose2d RIGHT_STARTING_POSE_RED =
      new Pose2d(
          FIELD_LENGTH - RIGHT_STARTING_POSE_BLUE.getX(),
          FIELD_HEIGHT - RIGHT_STARTING_POSE_BLUE.getY(),
          Rotation2d.kZero);

  public static final Pose2d LEFT_STATION_PICKUP_POSE_BLUE =
      new Pose2d(0.6, 7.85, Rotation2d.fromDegrees(-54));

  public static final Pose2d LEFT_STATION_PICKUP_POSE_RED =
      new Pose2d(
          FIELD_LENGTH - LEFT_STATION_PICKUP_POSE_BLUE.getX(),
          FIELD_HEIGHT - LEFT_STATION_PICKUP_POSE_BLUE.getY(),
          Rotation2d.fromDegrees(126));
  public static final Pose2d RIGHT_STATION_PICKUP_POSE_BLUE =
      new Pose2d(
          LEFT_STATION_PICKUP_POSE_BLUE.getX(),
          FIELD_HEIGHT - LEFT_STATION_PICKUP_POSE_BLUE.getY(),
          Rotation2d.fromDegrees(54));
  public static final Pose2d RIGHT_STATION_PICKUP_POSE_RED =
      new Pose2d(
          FIELD_LENGTH - RIGHT_STATION_PICKUP_POSE_BLUE.getX(),
          FIELD_HEIGHT - RIGHT_STATION_PICKUP_POSE_BLUE.getY(),
          Rotation2d.fromDegrees(-125));

  public static boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Blue;
  }

  public static boolean isOnBlueAlliance(Pose2d pose) {
    return pose.getX() < FIELD_LENGTH / 2.0;
  }

  public static Translation2d getMark1() {
    return isBlueAlliance() ? BLUE_MARK_1 : RED_MARK_1;
  }

  public static Translation2d getMark2() {
    return isBlueAlliance() ? BLUE_MARK_2 : RED_MARK_2;
  }

  public static Translation2d getMark3() {
    return isBlueAlliance() ? BLUE_MARK_3 : RED_MARK_3;
  }

  public static Pose2d getFarLeftStartingPose(DriverStation.Alliance alliance) {
    return alliance == DriverStation.Alliance.Blue
        ? FAR_LEFT_STARTING_POSE_BLUE
        : FAR_LEFT_STARTING_POSE_RED;
  }

  public static Pose2d getFarRightStartingPose(DriverStation.Alliance alliance) {
    return alliance == DriverStation.Alliance.Blue
        ? FAR_RIGHT_STARTING_POSE_BLUE
        : FAR_RIGHT_STARTING_POSE_RED;
  }

  public static Pose2d getLeftStartingPose(DriverStation.Alliance alliance) {
    return alliance == DriverStation.Alliance.Blue
        ? LEFT_STARTING_POSE_BLUE
        : LEFT_STARTING_POSE_RED;
  }

  public static Pose2d getRightStartingPose(DriverStation.Alliance alliance) {
    return alliance == DriverStation.Alliance.Blue
        ? RIGHT_STARTING_POSE_BLUE
        : RIGHT_STARTING_POSE_RED;
  }

  public static Pose2d getLeftStationPickup(DriverStation.Alliance alliance) {
    return alliance == DriverStation.Alliance.Blue
        ? LEFT_STATION_PICKUP_POSE_BLUE
        : LEFT_STATION_PICKUP_POSE_RED;
  }

  public static Pose2d getRightStationPickup(DriverStation.Alliance alliance) {
    return alliance == DriverStation.Alliance.Blue
        ? RIGHT_STATION_PICKUP_POSE_BLUE
        : RIGHT_STATION_PICKUP_POSE_RED;
  }

  public static Pose2d getFarLeftStartingPose() {
    return isBlueAlliance() ? FAR_LEFT_STARTING_POSE_BLUE : FAR_LEFT_STARTING_POSE_RED;
  }

  public static Pose2d getFarRightStartingPose() {
    return isBlueAlliance() ? FAR_RIGHT_STARTING_POSE_BLUE : FAR_RIGHT_STARTING_POSE_RED;
  }

  public static Pose2d getLeftStartingPose() {
    return isBlueAlliance() ? LEFT_STARTING_POSE_BLUE : LEFT_STARTING_POSE_RED;
  }

  public static Pose2d getRightStartingPose() {
    return isBlueAlliance() ? RIGHT_STARTING_POSE_BLUE : RIGHT_STARTING_POSE_RED;
  }

  public static Pose2d getLeftStationPickup() {
    return isBlueAlliance() ? LEFT_STATION_PICKUP_POSE_BLUE : LEFT_STATION_PICKUP_POSE_RED;
  }

  public static Pose2d getRightStationPickup() {
    return isBlueAlliance() ? RIGHT_STATION_PICKUP_POSE_BLUE : RIGHT_STATION_PICKUP_POSE_RED;
  }

  public static Pose3d getTagPose(int id) {
    if (id < RED_LEFT_CORAL_STATION || id > BLUE_REEF_LEFT_BARGE) {
      throw new IllegalArgumentException("id must be between 1 and 22");
    }

    return FIELD_LAYOUT
        .getTagPose(id)
        .orElseThrow(
            () -> {
              final String message = String.format("getTagPose called for unexpected tag %d", id);
              return new RuntimeException(message);
            });
  }

  public static Pose2d getBackoutPointToForL1Scoring(
      int tagID, ChoreographerConstants.ScoringSide scoringSide) {
    return getDesiredPointToDriveToForL1Scoring(
        tagID,
        scoringSide,
        Units.inchesToMeters(ChoreographerConstants.xOffsetFromTagForL1BackoutInches));
  }

  public static Pose2d getDesiredPointToDriveToForL1Scoring(
      int tagID, ChoreographerConstants.ScoringSide scoringSide) {
    return getDesiredPointToDriveToForL1Scoring(tagID, scoringSide, 0.0);
  }

  public static Pose2d getDesiredPointToDriveToForL1Scoring(
      int tagID,
      ChoreographerConstants.ScoringSide scoringSide,
      double distanceFromFinalScoringPose) {

    if (tagID >= 1 && tagID <= 22) {
      Pose2d tagPose = FieldConstants.FIELD_LAYOUT.getTagPose(tagID).get().toPose2d();
      double xOffset =
          Units.inchesToMeters(
              ChoreographerConstants.xOffsetFromTagForL1TopScoringInches
                  + Units.metersToInches(distanceFromFinalScoringPose));

      double yOffset =
          -Units.inchesToMeters(ChoreographerConstants.yOffsetFromTagForScoringL1Inches);

      Transform2d offsetFromTag = new Transform2d(xOffset, yOffset, Rotation2d.k180deg);

      Pose2d transformedPose = tagPose.plus(offsetFromTag);

      return transformedPose;
    } else {
      return Pose2d.kZero;
    }
  }

  public static Pose2d getDesiredFinalScoringPoseForCoral(
      int tagID,
      ChoreographerConstants.ScoringSide scoringSide,
      ChoreographerConstants.ScoringDirection scoringDirection) {
    return getDesiredPointToDriveToForCoralScoring(tagID, scoringSide, scoringDirection, 0.0);
  }

  public static Pose2d getDesiredIntermediateScoringPoseForCoral(
      int tagID,
      ChoreographerConstants.ScoringSide scoringSide,
      ChoreographerConstants.ScoringDirection scoringDirection) {
    return getDesiredPointToDriveToForCoralScoring(tagID, scoringSide, scoringDirection, 1);
  }

  public static Pose2d getDesiredPointToDriveToForCoralScoring(
      int tagID,
      ChoreographerConstants.ScoringSide scoringSide,
      ChoreographerConstants.ScoringDirection scoringDirection,
      double distanceFromFinalScoringPoseMeters) {

    if (tagID >= 1 && tagID <= 22) {
      Pose2d tagPose = FieldConstants.FIELD_LAYOUT.getTagPose(tagID).get().toPose2d();
      double xOffset =
          Units.inchesToMeters(
              ChoreographerConstants.xOffsetFromTagForScoringInches
                  + Units.metersToInches(distanceFromFinalScoringPoseMeters));

      double yOffset =
          -Units.inchesToMeters(ChoreographerConstants.yOffsetFromTagForScoringOnReefInchesBase);
      if (scoringSide == ChoreographerConstants.ScoringSide.RIGHT) {
        yOffset *= -1;
      }
      if (scoringDirection == ChoreographerConstants.ScoringDirection.LEFT) {
        yOffset += Units.inchesToMeters(ChoreographerConstants.yOffsetFromPoleForLeft);
      } else {
        yOffset -= Units.inchesToMeters(ChoreographerConstants.yOffsetFromPoleForLeft);
      }

      Translation2d offsetFromTag = new Translation2d(xOffset, yOffset);

      Rotation2d rotation = new Rotation2d();

      if (scoringDirection == ScoringDirection.LEFT) {
        rotation = Rotation2d.kCW_90deg;
      } else {
        rotation = Rotation2d.kCCW_90deg;
      }

      var transformedPose =
          tagPose.plus(new Transform2d(offsetFromTag.getX(), offsetFromTag.getY(), rotation));

      return transformedPose;
    } else {
      return Pose2d.kZero;
    }
  }

  public static Pose2d getClosestStation(Pose2d robotPose) {
    Pose2d[] stations = {
      LEFT_STATION_PICKUP_POSE_BLUE,
      RIGHT_STATION_PICKUP_POSE_BLUE,
      LEFT_STATION_PICKUP_POSE_RED,
      RIGHT_STATION_PICKUP_POSE_RED
    };

    Pose2d closestStation = stations[0];
    double minDistance = robotPose.getTranslation().getDistance(stations[0].getTranslation());

    for (int i = 1; i < stations.length; i++) {
      double distance = robotPose.getTranslation().getDistance(stations[i].getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestStation = stations[i];
      }
    }

    return closestStation;
  }
}
