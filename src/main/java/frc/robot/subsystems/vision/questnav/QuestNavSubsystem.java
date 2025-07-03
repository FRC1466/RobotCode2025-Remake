// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision.questnav;

import static frc.robot.subsystems.vision.questnav.QuestNavConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.classical.Vision.VisionConsumer;
import frc.robot.subsystems.vision.questnav.io.QuestNavIO;
import frc.robot.subsystems.vision.questnav.io.QuestNavIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Manages communication and pose estimation with a Meta Quest VR headset.
 *
 * <p>This subsystem leverages the Quest's inside-out SLAM tracking system to provide high-frequency
 * (120Hz) robot pose estimation. Key features:
 *
 * <p>- Global SLAM-based localization - Field mapping and persistence - Sub-centimeter tracking
 * precision - High update rate (120Hz) - Drift-free position tracking - Fast relocalization
 *
 * <p>The system operates in phases: 1. Pre-match mapping to capture field features 2. Initial pose
 * acquisition and alignment 3. Continuous pose updates during match 4. Recovery handling if
 * tracking is lost
 */
public class QuestNavSubsystem extends SubsystemBase {
  /** Hardware communication interface */
  private final QuestNavIO io;

  /** Consumer for pose updates from the QuestNav */
  private final VisionConsumer visionConsumer;

  /** Logged inputs from Quest hardware */
  private final QuestNavIOInputsAutoLogged inputs = new QuestNavIOInputsAutoLogged();

  /**
   * Creates a new QuestNavSubsystem.
   *
   * <p>Initializes communication with Quest hardware and prepares logging systems. The subsystem
   * starts in an uninitialized state requiring pose calibration.
   *
   * @param questNavConsumer Consumer that receives pose updates from the headset
   * @param io Interface for Quest hardware communication
   */
  public QuestNavSubsystem(VisionConsumer questNavConsumer, QuestNavIO io) {
    this.io = io;
    this.visionConsumer = questNavConsumer;
    Logger.recordOutput("QuestNav/status", "Initialized");
  }

  /**
   * Updates subsystem state and processes Quest data.
   *
   * <p>Called periodically by the command scheduler. This method: - Updates hardware inputs -
   * Processes new pose data - Handles state transitions - Manages reset operations - Updates
   * logging
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("QuestNav", inputs);

    // Add to Kalman filter
    processPose();

    // Notify if we are disconnected
    Logger.recordOutput("QuestNav/Connected", inputs.connected);
    Logger.recordOutput("QuestNav/BatteryPercent", inputs.batteryPercent);
    Logger.recordOutput("QuestNav/CurrentlyTracking", inputs.currentlyTracking);
    Logger.recordOutput("QuestNav/TrackingLostCounter", inputs.trackingLostCounter);
    Logger.recordOutput("QuestNav/Timestamp", inputs.timestamp);
  }

  /**
   * Returns the battery percentage of the connected Quest headset.
   *
   * @return Battery percentage (0-100)
   */
  public double getBatteryPercent() {
    return inputs.batteryPercent;
  }

  /**
   * Returns the timestamp of the most recent pose update.
   *
   * @return Timestamp in seconds
   */
  public double getTimestamp() {
    return inputs.timestamp;
  }

  /**
   * Checks if the Quest headset is currently connected.
   *
   * @return True if connected, false otherwise
   */
  public boolean isConnected() {
    return inputs.connected;
  }

  /**
   * Gets the current robot pose as estimated by the Quest headset. This incorporates all transforms
   * and offsets to convert from headset to robot coordinates.
   *
   * @return Field-relative robot pose
   */
  @AutoLogOutput(key = "QuestNav/Pose")
  public Pose2d getPose() {
    return getQuestNavPose().transformBy(ROBOT_TO_QUESTNAV.inverse());
  }

  /**
   * Resets the pose tracking system to a specified position. Must be called only when the robot is
   * disabled to avoid interrupting tracking during a match.
   *
   * @param pose The new reference pose
   */
  public void resetPose(Pose2d pose) {
    // Transform the pose to the QuestNav coordinate system w/ offset
    Pose2d questNavSidePose = pose.plus(ROBOT_TO_QUESTNAV);

    // Send the request
    io.setPose(questNavSidePose);

    Logger.recordOutput(
        "QuestNav/Log",
        String.format("Resetting pose to WPILib: %s, QuestNav: %s", pose, questNavSidePose));
  }

  private final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /**
   * Processes the current pose data and forwards it to the consumer if connected and properly
   * tracking. This enables integration with pose estimation systems.
   */
  private void processPose() {
    if (inputs.connected && inputs.currentlyTracking) {
      Pose2d pose = getPose();
      double timestamp = getTimestamp();

      // Make sure we are inside the field
      if (pose.getX() < 0.0
          || pose.getX() > aprilTagFieldLayout.getFieldLength()
          || pose.getY() < 0.0
          || pose.getY() > aprilTagFieldLayout.getFieldWidth()) {
        return;
      }
        visionConsumer.accept(pose, timestamp, QUESTNAV_STD_DEVS);
    }
  }

  /**
   * Combines QuestNav position and orientation into a unified Pose2d.
   *
   * @return Raw Pose2d from the headset's perspective
   */
  @AutoLogOutput(key = "QuestNav/RawPose")
  private Pose2d getQuestNavPose() {
    return inputs.pose2d;
  }
}
