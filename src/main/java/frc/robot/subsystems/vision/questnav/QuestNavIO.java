// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for handling input/output operations with the Oculus Quest hardware. */
public interface QuestNavIO {
  /** Data structure for Oculus inputs that can be automatically logged. */
  @AutoLog
  public static class QuestNavIOInputs {
    public boolean connected = false;

    public boolean SimReal = false;

    /** 3D position coordinates [x, y, z] */
    public float[] position = new float[] {0.0f, 0.0f, 0.0f};

    /** Quaternion orientation [w, x, y, z] */
    public float[] quaternion = new float[] {0.0f, 0.0f, 0.0f, 0.0f};

    /** Euler angles [roll, pitch, yaw] in degrees */
    public float[] eulerAngles = new float[] {0.0f, 0.0f, 0.0f};

    /** Current timestamp from the Oculus */
    public double timestamp = -1.0;

    /** Frame counter from the Oculus */
    public int frameCount = -1;

    /** Battery level percentage */
    public double batteryPercent = -1.0;

    /** Current MISO (Master In Slave Out) value */
    public int misoValue = 0;

    /** Does the Oculus have 6dof tracking? */
    public boolean isTracking = false;

    /** Total number of tracking lost events since the Quest has booted */
    public int totalTrackingLostEvents = 0;
  }

  /**
   * Updates the set of loggable inputs from the Oculus.
   *
   * @param inputs The input object to update with current values
   */
  public default void updateInputs(QuestNavIOInputs inputs) {}

  /**
   * Resets the pose components for resetting the Oculus position tracking. HARD RESET.
   *
   * @param oculusTargetPose The target pose of the oculus to reset to. NOT THE TARGET ROBOT POSE
   */
  public default void resetPose(Pose2d oculusTargetPose) {}

  /** Resets the current heading of the Oculus as zero */
  public default void resetHeading() {}
}
