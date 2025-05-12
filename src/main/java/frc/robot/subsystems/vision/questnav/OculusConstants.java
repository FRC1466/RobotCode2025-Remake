// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision.questnav;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import lombok.experimental.UtilityClass;

/**
 * Constants used by the Oculus Quest navigation subsystem. Contains configuration values for
 * physical setup and operation parameters.
 */
@UtilityClass
public class OculusConstants {

  /**
   * Determines which side the pose reset logic should be used
   *
   * <p>ROBOT_SIDE -> All resets happen with a transform applied to the robot code. resetPose() must
   * be called ONCE prior to the start of the match to avoid latency in resetting the Oculus side
   * pose to (0, 0, 0) Afterward, updateTransform() should be called instead if the transform needs
   * to be updated.
   *
   * <p>OCULUS_SIDE -> All resets happen with a transform applied to the Oculus code. These methods
   * should NOT be called during match play as it will result in incorrect transformations due to
   * latency.
   */
  public static enum PoseResetStrategy {
    ROBOT_SIDE,
    OCULUS_SIDE
  }

  /** The strategy used to reset the pose */
  public static final PoseResetStrategy POSE_RESET_STRATEGY = PoseResetStrategy.OCULUS_SIDE;

  public static final double BATTERY_LOW_PERCENT = 20;

  public static final double BATTERY_CRITICAL_PERCENT = 10;

  /**
   * Transform from the robot center to the headset. Coordinate system: - X: Positive is forwards -
   * Y: Positive is left - Rotation: Positive is counter-clockwise
   */
  public static final Transform2d ROBOT_TO_OCULUS =
      new Transform2d(0, 0, Rotation2d.fromDegrees(0));

  /** Timeout threshold for considering Quest disconnected (seconds) */
  public static final Time OCULUS_CONNECTION_TIMEOUT = Milliseconds.of(350);

  /**
   * Standard deviations representing how much we "trust" the position from the Oculus. By default,
   * the Quest 3 provides sub-centimeter accuracy. Values represent: [0]: X position trust (50mm)
   * [1]: Y position trust (50mm) [2]: Rotation trust (~5 degrees)
   */
  public static final Matrix<N3, N1> OCULUS_STD_DEVS =
      VecBuilder.fill(
          0.02, // Trust down to 50mm
          0.02, // Trust down to 50mm
          0.0872665 // 5deg
          );
}
