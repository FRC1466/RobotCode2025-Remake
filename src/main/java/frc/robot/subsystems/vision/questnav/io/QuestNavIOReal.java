// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision.questnav.io;

import edu.wpi.first.math.geometry.Pose2d;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;

/** Implementation of questNavIO for real hardware communication via QuestNav vendor library. */
public class QuestNavIOReal implements QuestNavIO {
  /** QuestNav instance for communication with the Quest headset */
  private final QuestNav questNav;

  /** Creates a new questNavIOReal instance using the QuestNav vendor library. */
  public QuestNavIOReal() {
    questNav = new QuestNav();
  }

  @Override
  public void updateInputs(QuestNavIOInputs inputs) {
    // Update connection status
    inputs.connected = questNav.isConnected();

    // Update frame data
    inputs.frameCount = questNav.getFrameCount();
    inputs.pose2d = questNav.getPose();

    // Update device data
    inputs.batteryPercent = questNav.getBatteryPercent();
    inputs.currentlyTracking = questNav.isTracking();
    inputs.trackingLostCounter = questNav.getTrackingLostCounter();

    // Other data
    inputs.latency = questNav.getLatency();

    // Process any pending command responses
    questNav.commandPeriodic();
  }

  @Override
  public void setPose(Pose2d questNavTargetPose) {
    questNav.setPose(questNavTargetPose);
    Logger.recordOutput(
        "QuestNav/Log",
        String.format(
            "Pose reset requested to: (%.2f, %.2f, %.2f°)",
            questNavTargetPose.getX(),
            questNavTargetPose.getY(),
            questNavTargetPose.getRotation().getDegrees()));
  }
}
