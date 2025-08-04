// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Map;

public class VisionConstants {
  // AprilTag field layout
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera configuration record
  public record CameraConfig(String name, Transform3d robotToCamera) {}

  // Camera configurations by robot type
  public static final Map<Integer, CameraConfig> compCameras =
      Map.of(
          0,
              new CameraConfig(
                  "Camera_FrontLeft",
                  new Transform3d(new Translation3d(.267, .292, .2), new Rotation3d(0, 0, 0))),
          1,
              new CameraConfig(
                  "Camera_FrontRight",
                  new Transform3d(new Translation3d(.267, -.292, .2), new Rotation3d(0, 0, 0))));

  public static final Map<Integer, CameraConfig> devCameras =
      Map.of(
          0,
          new CameraConfig(
              "Camera_DevBot",
              new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0))));

  // Simulation camera configuration
  public enum SimCameraMode {
    CompCameras,
    DevCameras,
    NoCameras
  }

  public static final SimCameraMode simCameraMode = SimCameraMode.CompCameras;

  public static final Map<Integer, CameraConfig> simCameras =
      switch (simCameraMode) {
        case CompCameras -> compCameras;
        case DevCameras -> devCameras;
        case NoCameras -> Map.of();
      };

  // Camera map getter based on current robot type
  public static Map<Integer, CameraConfig> cameras = getCameraMap();

  private static Map<Integer, CameraConfig> getCameraMap() {
    return switch (frc.robot.constants.Constants.getRobot()) {
      case COMPBOT -> compCameras;
      case DEVBOT -> devCameras;
      case SIMBOT -> simCameras;
    };
  }

  // Vision filtering thresholds
  public static final double maxAmbiguity = 0.3;
  public static final double maxZError = 0.75;

  // Standard deviation baselines for pose estimation
  // (For 1 meter distance and 1 tag, adjusted automatically based on distance and tag count)
  public static final double linearStdDevBaseline = 0.04; // Meters
  public static final double angularStdDevBaseline = 0.06; // Radians

  // Camera-specific standard deviation multipliers
  // (Adjust to trust some cameras more than others)
  public static final double[] cameraStdDevFactors = {
    4.0, // Camera 0
    4.0 // Camera 1
  };

  // MegaTag 2 standard deviation multipliers
  public static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static final double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
