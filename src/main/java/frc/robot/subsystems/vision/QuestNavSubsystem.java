// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
  QuestNav questNav = new QuestNav();
  VisionConsumer consumer;

  public QuestNavSubsystem(VisionConsumer consumer) {
    this.consumer = consumer;
  }

  public Pose2d getPose() {
    Pose2d questPose = questNav.getPose();
    Pose2d robotPose = questPose.transformBy(QUEST_TO_ROBOT.inverse());
    return robotPose;
  }

  @Override
  public void periodic() {
    questNav.cleanupResponses();
    questNav.processHeartbeat();

    if (questNav.getConnected() && questNav.getTrackingStatus()) {
      Pose2d pose = getPose();
      double timestamp = questNav.getTimestamp();
      consumer.accept(pose, timestamp, QUESTNAV_STD_DEVS);
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
