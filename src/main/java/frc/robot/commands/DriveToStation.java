// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Reef;
import frc.robot.RobotState;
import frc.robot.subsystems.drive2.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class DriveToStation extends Command {
  private static final LoggedTunableNumber stationAlignDistance =
      new LoggedTunableNumber(
          "DriveToStation/StationAlignDistance", Drive.robotWidth / 2.0 + Units.inchesToMeters(8));
  private static final LoggedTunableNumber horizontalMaxOffset =
      new LoggedTunableNumber(
          "DriveToStation/HorizontalMaxOffset",
          FieldConstants.CoralStation.stationLength / 2 - Units.inchesToMeters(25));
  private static final LoggedTunableNumber autoOffset =
      new LoggedTunableNumber(
          "DriveToStation/AutoOffset",
          FieldConstants.CoralStation.stationLength / 2 - Units.inchesToMeters(24));

  private final Drive drive;
  private final boolean isAuto;

  public DriveToStation(Drive drive, boolean isAuto) {
    this.drive = drive;
    this.isAuto = isAuto;
  }

  public void execute() {
    Pose2d robot =
        AllianceFlipUtil.apply(RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry());
    List<Pose2d> finalPoses = new ArrayList<>();
    for (Pose2d stationCenter :
        new Pose2d[] {
          FieldConstants.CoralStation.leftCenterFace, FieldConstants.CoralStation.rightCenterFace
        }) {
      Transform2d offset = new Transform2d(stationCenter, robot);
      offset =
          new Transform2d(
              stationAlignDistance.get(),
              isAuto
                  ? (robot.getY() < FieldConstants.fieldWidth / 2.0
                      ? -autoOffset.get()
                      : autoOffset.get())
                  : MathUtil.clamp(
                      offset.getY(), -horizontalMaxOffset.get(), horizontalMaxOffset.get()),
              Rotation2d.kZero);

      finalPoses.add(stationCenter.transformBy(offset));
    }
    Pose2d intakePose = AllianceFlipUtil.apply(robot.nearest(finalPoses));
    robot = AllianceFlipUtil.apply(robot);
    Pose2d goal;
    if (withinDistanceToReef(robot, 0.35)) {
      final double yError = intakePose.relativeTo(robot).getY();
      goal =
          robot
              .transformBy(
                  GeomUtil.toTransform2d(-3.0, Math.abs(yError) > 0.6 ? yError * 0.6 : yError))
              .transformBy(
                  GeomUtil.toTransform2d(
                      robot
                          .getRotation()
                          .interpolate(intakePose.getRotation(), 0.05)
                          .minus(robot.getRotation())));
    } else {
      goal = intakePose;
    }

    Logger.recordOutput(
        "DriveToStation/LeftClosestPose", AllianceFlipUtil.apply(finalPoses.get(0)));
    Logger.recordOutput(
        "DriveToStation/RightClosestPose", AllianceFlipUtil.apply(finalPoses.get(1)));

    drive.setDesiredPoseForDriveToPoint(goal);
  }

  public boolean withinTolerance(double translationTolerance, Rotation2d rotationTolerance) {
    return drive.isAtDriveToPointSetpoint(translationTolerance, rotationTolerance);
  }

  private static final double reefRadius = Reef.faceLength;

  public static boolean withinDistanceToReef(Pose2d robot, double distance) {
    final double distanceToReefCenter =
        AllianceFlipUtil.apply(robot).getTranslation().getDistance(Reef.center);
    Logger.recordOutput("DriveToScore/DistanceToReefCenter", distanceToReefCenter);
    return distanceToReefCenter <= reefRadius + Drive.robotWidth / 2.0 + distance;
  }
}
