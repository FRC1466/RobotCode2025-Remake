// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision.questnav.io;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for handling input/output operations with the questNav Quest hardware. */
public interface QuestNavIO {
  /** Data structure for questNav inputs that can be automatically logged. */
  @AutoLog
  public static class QuestNavIOInputs {

    public boolean connected = false;

    /** Frame counter from the questNav */
    public int frameCount = -1;

    /** Current timestamp from the questNav */
    public double timestamp = -1.0;

    /** Current pose from the questNav */
    public Pose2d pose2d = Pose2d.kZero;

    /** Total number of tracking lost events since the Quest has booted */
    public int trackingLostCounter = 0;

    /** Does the questNav have 6dof tracking? */
    public boolean currentlyTracking = false;

    /** Battery level percentage */
    public double batteryPercent = -1.0;

    /** Quest > Robot Latency in MS */
    public double latency = -1.0;
  }

  /**
   * Updates the set of loggable inputs from the questNav.
   *
   * @param inputs The input object to update with current values
   */
  public default void updateInputs(QuestNavIOInputs inputs) {}

  /**
   * Resets the pose components for resetting the questNav position tracking. HARD RESET.
   *
   * @param questNavTargetPose The target pose of the questNav to reset to. NOT THE TARGET ROBOT
   *     POSE
   */
  public default void setPose(Pose2d questNavTargetPose) {}
}
