// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StructArrayPublisher;

public class MechanismVisualizer implements AutoCloseable {
  // Poses: 0 coral, 1 algae, 2 diff base, 3 wrist, 4..N elevator stages (top to base)
  private final StructArrayPublisher<Pose3d> mechanismPosesPublisher;
  private final Pose3d[] poses;

  // Geometry (robot frame)
  private static final Translation3d coralPivotAnchor = new Translation3d(0.330007, 0.0, 0.215652);
  private static final Translation3d algaePivotAnchor = new Translation3d(0.330007, 0.0, 0.215652);
  private static final Translation3d elevatorBaseAnchor =
      new Translation3d(-0.179166, 0.0, 0.279399);

  // Diff mount relative to elevator base frame (when height=0, rotation=0)
  private static final Translation3d diffMountOffsetLocal =
      new Translation3d(-0.091834, -0.005000, 0.528601);

  // Elevator configuration
  private static final int elevatorStageCount = 5; // includes base-most fixed stage
  private static final int movingStages = Math.max(1, elevatorStageCount - 1);
  private static final double defaultTotalMaxTravel = 0.60;
  private static final double defaultPerStageTravel = defaultTotalMaxTravel / movingStages;

  // Per-stage motion
  private final double[] perStageTravel = new double[movingStages]; // top-most first
  private final double[] stageZeroOffsets = new double[elevatorStageCount]; // top-most first

  public MechanismVisualizer(NetworkTable table) {
    poses = new Pose3d[2 + 2 + elevatorStageCount];
    for (int i = 0; i < poses.length; i++) poses[i] = new Pose3d();

    for (int i = 0; i < movingStages; i++) perStageTravel[i] = defaultPerStageTravel;
    // stageZeroOffsets default to 0

    mechanismPosesPublisher = table.getStructArrayTopic("MechanismPoses", Pose3d.struct).publish();
    mechanismPosesPublisher.set(poses);
  }

  // Update all mechanism poses and publish
  public void update(
      Pose2d robotPose,
      double slapdownCoral,
      double slapdownAlgae,
      double diffRotation,
      double diffPivot,
      double elevatorHeight,
      double elevatorRotation) {

    // Slapdowns (about local Y)
    poses[0] = new Pose3d(coralPivotAnchor, new Rotation3d(0, slapdownCoral, 0));
    poses[1] = new Pose3d(algaePivotAnchor, new Rotation3d(0, slapdownAlgae, 0));

    // Elevator base (pitch about Y)
    Rotation3d rotElev = new Rotation3d(0, elevatorRotation, 0);
    Pose3d elevBasePose = new Pose3d(elevatorBaseAnchor, rotElev);

    // Differential base position (local +Z extension)
    Translation3d diffLocal = diffMountOffsetLocal.plus(new Translation3d(0, 0, elevatorHeight));
    Pose3d diffBasePose = elevBasePose.transformBy(new Transform3d(diffLocal, new Rotation3d()));

    // Differential rotations: elevator tilt -> pivot -> wrist spin
    Rotation3d diffBaseRot = new Rotation3d(0, diffPivot, 0).rotateBy(rotElev);
    Rotation3d wristRot = new Rotation3d(0, 0, diffRotation).rotateBy(diffBaseRot);

    poses[2] = new Pose3d(diffBasePose.getTranslation(), diffBaseRot);
    poses[3] = new Pose3d(diffBasePose.getTranslation(), wristRot);

    // Elevator stages (continuous sequential extension)
    double totalMax = 0.0;
    for (int j = 0; j < movingStages; j++) totalMax += Math.max(0.0, perStageTravel[j]);
    double E = clamp(elevatorHeight, 0.0, totalMax);

    double[] slotStart = new double[movingStages];
    double acc = 0.0;
    for (int j = 0; j < movingStages; j++) {
      slotStart[j] = acc;
      acc += Math.max(0.0, perStageTravel[j]);
    }

    for (int i = 0; i < elevatorStageCount; i++) {
      double stageZ;
      if (i == elevatorStageCount - 1) {
        stageZ = stageZeroOffsets[i];
      } else {
        double sum = 0.0;
        for (int j = i; j < movingStages; j++) {
          double len = Math.max(0.0, perStageTravel[j]);
          double ej = clamp(E - slotStart[j], 0.0, len);
          sum += ej;
        }
        stageZ = stageZeroOffsets[i] + sum;
      }
      Pose3d stagePose =
          elevBasePose.transformBy(
              new Transform3d(new Translation3d(0, 0, stageZ), new Rotation3d()));
      poses[4 + i] = stagePose;
    }

    // Force base-most to anchor (no translation drift)
    poses[4 + elevatorStageCount - 1] = new Pose3d(elevatorBaseAnchor, rotElev);

    // Convert robot-local to field
    Pose3d robotPose3d =
        new Pose3d(
            robotPose.getX(),
            robotPose.getY(),
            0.0,
            new Rotation3d(0, 0, robotPose.getRotation().getRadians()));

    for (int i = 0; i < poses.length; i++) {
      poses[i] =
          robotPose3d.transformBy(
              new Transform3d(poses[i].getTranslation(), poses[i].getRotation()));
    }

    mechanismPosesPublisher.set(poses);
  }

  public void setStageTravel(double[] perStageTravelMeters) {
    if (perStageTravelMeters == null || perStageTravelMeters.length != movingStages) return;
    for (int i = 0; i < movingStages; i++)
      perStageTravel[i] = Math.max(0.0, perStageTravelMeters[i]);
  }

  public void setStageZeroOffsets(double[] stageZeroOffsetsMeters) {
    if (stageZeroOffsetsMeters == null || stageZeroOffsetsMeters.length != elevatorStageCount)
      return;
    for (int i = 0; i < elevatorStageCount; i++) stageZeroOffsets[i] = stageZeroOffsetsMeters[i];
  }

  private static double clamp(double v, double min, double max) {
    return Math.max(min, Math.min(v, max));
  }

  @Override
  public void close() {
    try {
      mechanismPosesPublisher.close();
    } catch (Exception ignored) {
    }
  }
}
