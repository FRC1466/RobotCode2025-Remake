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
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class MechanismVisualizer {
  // Poses: 0 coral, 1 algae, 2 diff base, 3 wrist, 4..N elevator stages (top to base)
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

  // Tunable offsets for hasCoral and hasAlgae
  private final LoggedTunableNumber hasCoralOffsetX =
      new LoggedTunableNumber("Mechanism/HasCoralOffsetX", 0.0);
  private final LoggedTunableNumber hasCoralOffsetY =
      new LoggedTunableNumber("Mechanism/HasCoralOffsetY", 0.0);
  private final LoggedTunableNumber hasCoralOffsetZ =
      new LoggedTunableNumber("Mechanism/HasCoralOffsetZ", 0.25);
  private final LoggedTunableNumber hasCoralRotX =
      new LoggedTunableNumber("Mechanism/HasCoralRotX", 0.0);
  private final LoggedTunableNumber hasCoralRotY =
      new LoggedTunableNumber("Mechanism/HasCoralRotY", 0.0);
  private final LoggedTunableNumber hasCoralRotZ =
      new LoggedTunableNumber("Mechanism/HasCoralRotZ", 0.0);

  private final LoggedTunableNumber hasCoralSlapdownOffsetX =
      new LoggedTunableNumber("Mechanism/HasCoralSlapdownOffsetX", -.275);
  private final LoggedTunableNumber hasCoralSlapdownOffsetY =
      new LoggedTunableNumber("Mechanism/HasCoralSlapdownOffsetY", 0.0);
  private final LoggedTunableNumber hasCoralSlapdownOffsetZ =
      new LoggedTunableNumber("Mechanism/HasCoralSlapdownOffsetZ", -.05);
  private final LoggedTunableNumber hasCoralSlapdownRotX =
      new LoggedTunableNumber("Mechanism/HasCoralSlapdownRotX", 0.0);
  private final LoggedTunableNumber hasCoralSlapdownRotY =
      new LoggedTunableNumber("Mechanism/HasCoralSlapdownRotY", 0.0);
  private final LoggedTunableNumber hasCoralSlapdownRotZ =
      new LoggedTunableNumber("Mechanism/HasCoralSlapdownRotZ", 1.57);

  private final LoggedTunableNumber hasAlgaeOffsetX =
      new LoggedTunableNumber("Mechanism/HasAlgaeOffsetX", -0.005);
  private final LoggedTunableNumber hasAlgaeOffsetY =
      new LoggedTunableNumber("Mechanism/HasAlgaeOffsetY", 0.0);
  private final LoggedTunableNumber hasAlgaeOffsetZ =
      new LoggedTunableNumber("Mechanism/HasAlgaeOffsetZ", 0.4);

  public MechanismVisualizer() {
    poses = new Pose3d[2 + 2 + elevatorStageCount];
    for (int i = 0; i < poses.length; i++) poses[i] = new Pose3d();

    for (int i = 0; i < movingStages; i++) perStageTravel[i] = defaultPerStageTravel;
    // stageZeroOffsets default to 0
  }

  // Update all mechanism poses and log
  public void update(
      Pose2d robotPose,
      double slapdownCoral,
      double slapdownAlgae,
      double diffRotation,
      double diffPivot,
      double elevatorHeight,
      double elevatorRotation,
      boolean hasCoral,
      boolean coralInSlapdown,
      boolean hasAlgae) {

    // Slapdowns (initial local pose around anchor; rotation applied later for visualization)
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
    Rotation3d wristRot =
        new Rotation3d(0, 0, diffRotation + Units.degreesToRadians(2.5)).rotateBy(diffBaseRot);

    poses[2] = new Pose3d(diffBasePose.getTranslation(), diffBaseRot);
    poses[3] = new Pose3d(diffBasePose.getTranslation(), wristRot);

    // Elevator stages (continuous sequential extension)
    double totalMax = 0.0;
    for (int j = 0; j < movingStages; j++) totalMax += Math.max(0.0, perStageTravel[j]);
    double E = clamp(elevatorHeight, 0.0, totalMax);

    double[] slotStart = new double[movingStages];
    double acc = 0.0;
    for (int j = movingStages - 1; j >= 0; j--) {
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

      poses[4 + i] =
          elevBasePose.transformBy(
              new Transform3d(new Translation3d(0, 0, stageZ), new Rotation3d()));
    }

    // Force base-most to anchor (no translation drift)
    poses[4 + elevatorStageCount - 1] = new Pose3d(elevatorBaseAnchor, rotElev);

    // ---- Logging (replacement block) ----
    Logger.recordOutput("MechanismVisualization/MechanismPoses", poses);

    // Convert robot 2D pose -> 3D world pose once
    Pose3d robot3d = new Pose3d(robotPose);

    // wrist pose is computed in robot frame as poses[3].
    Pose3d wristRobotFrame = poses[3]; // pivot point & wrist rotation in robot frame

    // Convert wrist into world coordinates (robot -> world)
    Pose3d wristWorld =
        robot3d.transformBy(
            new Transform3d(wristRobotFrame.getTranslation(), wristRobotFrame.getRotation()));

    // ---------- CORAL ----------
    if (hasCoral) {
      if (coralInSlapdown) {
        Pose3d anchorRobotPose = poses[0];

        Pose3d anchorWorld =
            robot3d.transformBy(
                new Transform3d(anchorRobotPose.getTranslation(), anchorRobotPose.getRotation()));

        Translation3d slapLocalOffset =
            new Translation3d(
                hasCoralSlapdownOffsetX.get(),
                hasCoralSlapdownOffsetY.get(),
                hasCoralSlapdownOffsetZ.get());

        Pose3d placed = anchorWorld.transformBy(new Transform3d(slapLocalOffset, new Rotation3d()));

        // Coral’s own tunable rotation
        Rotation3d coralRot =
            new Rotation3d(
                hasCoralSlapdownRotX.get(), hasCoralSlapdownRotY.get(), hasCoralSlapdownRotZ.get());

        // Add the robot’s yaw to coral yaw
        double robotYaw = robot3d.getRotation().getZ(); // robot heading
        Rotation3d finalRot =
            new Rotation3d(coralRot.getX(), coralRot.getY(), coralRot.getZ() + robotYaw);

        Pose3d slapWorld = new Pose3d(placed.getTranslation(), finalRot);

        Logger.recordOutput("MechanismVisualization/HasCoralPose", new Pose3d[] {slapWorld});
      } else {
        // CORAL IN CLAW (attach to wrist world pose; translation/rotation are local to wrist)
        Translation3d coralLocal =
            new Translation3d(hasCoralOffsetX.get(), hasCoralOffsetY.get(), hasCoralOffsetZ.get());

        Rotation3d coralLocalRot =
            new Rotation3d(hasCoralRotX.get(), hasCoralRotY.get(), hasCoralRotZ.get());

        Pose3d coralClawWorld = wristWorld.transformBy(new Transform3d(coralLocal, coralLocalRot));

        Logger.recordOutput("MechanismVisualization/HasCoralPose", new Pose3d[] {coralClawWorld});
      }
    } else {
      Logger.recordOutput("MechanismVisualization/HasCoralPose", new Pose3d[] {});
    }

    // ---------- ALGAE ----------
    if (hasAlgae) {
      Translation3d algaeLocal =
          new Translation3d(hasAlgaeOffsetX.get(), hasAlgaeOffsetY.get(), hasAlgaeOffsetZ.get());

      Rotation3d algaeLocalRot = new Rotation3d(0.0, 0.0, 0.0); // add tunables if desired

      Pose3d algaeClawWorld = wristWorld.transformBy(new Transform3d(algaeLocal, algaeLocalRot));

      Logger.recordOutput(
          "MechanismVisualization/HasAlgaePose",
          new Translation3d[] {algaeClawWorld.getTranslation()});
    } else {
      Logger.recordOutput("MechanismVisualization/HasAlgaePose", new Translation3d[] {});
    }
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
}
