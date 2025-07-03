// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static frc.robot.commands.DriveCommands.DEADBAND;
import static frc.robot.commands.DriveToStation.withinDistanceToReef;
import static frc.robot.subsystems.vision.classical.VisionConstants.aprilTagLayout;
import static frc.robot.util.EqualsUtil.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.PathfindConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FlipField;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToScore extends DriveToPose {
  private static final LoggedTunableNumber ApproachEpsilon =
      new LoggedTunableNumber("DriveToScore/ApproachEpsilon", 0.1);
  private static final LoggedTunableNumber AngularAproachEpsilon =
      new LoggedTunableNumber("DriveToScore/AngularAproachEpsilon", 0.1);
  private static LoggedTunableNumber approachDistance =
      new LoggedTunableNumber("DriveToScore/ApproachDistance", .7);

  public DriveToScore(Drive drive, boolean isAuto, Supplier<Integer> coralPovSupplier) {
    this(drive, () -> 0.0, () -> 0.0, () -> 0.0, isAuto, coralPovSupplier);
  }

  public DriveToScore(
      Drive drive,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      boolean isAuto,
      Supplier<Integer> coralPovSupplier) {
    this(
        drive,
        () ->
            DriveCommands.getLinearVelocityFromJoysticks(
                    driverX.getAsDouble(), driverY.getAsDouble())
                .times(FlipField.shouldFlip() ? -1.0 : 1.0),
        () ->
            Math.copySign(
                Math.pow(MathUtil.applyDeadband(driverOmega.getAsDouble(), DEADBAND), 2.0),
                driverOmega.getAsDouble()),
        isAuto,
        () -> {
          int pov = coralPovSupplier.get();
          if (FlipField.shouldFlip()) {
            pov = 1 - pov;
          }
          return pov;
        });
  }

  public DriveToScore(
      Drive drive,
      Supplier<Translation2d> linearFF,
      DoubleSupplier theta,
      boolean isAuto,
      Supplier<Integer> coralPovSupplier) {
    super(
        drive,
        () -> {
          int tag = getClosestTag(drive);
          int pov = coralPovSupplier.get();
          // choose approach and real goal
          Pose2d approach = PathfindConstants.getTargetPoseReefApproach()[tag][pov];
          Pose2d target = PathfindConstants.getTargetPoseReef()[tag][pov];
          Pose2d robot = drive.getPose();
          double distance = robot.getTranslation().getDistance(approach.getTranslation());
          double angularDistance =
              Math.abs(
                  MathUtil.angleModulus(
                      robot.getRotation().getRadians() - approach.getRotation().getRadians()));
          double currentApproachDistance = approachDistance.get();
          double currentApproachEpsilon = ApproachEpsilon.get();
          double currentAngularApproachEpsilon = AngularAproachEpsilon.get();
          Logger.recordOutput(
              "DriveToScore/withinReefApproachDistance",
              withinDistanceToReef(robot, currentApproachDistance));

          if (withinDistanceToReef(robot, currentApproachDistance)
              || (epsilonEquals(distance, 0, currentApproachEpsilon)
                  && epsilonEquals(angularDistance, 0, currentAngularApproachEpsilon))) {
            return target;
          }
          return approach;
        },
        drive::getPose,
        linearFF,
        theta);
  }

  public static int getClosestTag(Drive drive) {
    int closestTag = -1;
    double prevDistance = Double.MAX_VALUE;
    double holderDistance = 0;
    int offset = 0;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        offset = 6;
      } else if (alliance.get() == Alliance.Blue) {
        offset = 17;
      }
    } else {
      // Default behavior if alliance is unknown
      offset = 0; // or whatever default makes sense
    }
    try {
      for (int i = offset; i < (offset + 6); i++) {
        Optional<Pose3d> tagPoseOptional = aprilTagLayout.getTagPose(i);
        if (tagPoseOptional.isEmpty()) {
          continue;
        }
        // Check if the Optional contains a value before calling get()
        if (tagPoseOptional.isPresent()) {
          var tagPose = tagPoseOptional.get();
          holderDistance =
              (Math.pow(drive.getPose().getX() - tagPose.getX(), 2)
                  + Math.pow(drive.getPose().getY() - tagPose.getY(), 2));
          if (holderDistance < prevDistance) {
            closestTag = i;
            prevDistance = holderDistance;
          }
        }
      }
    } catch (Exception e) {
      // Retrieve the actual exception thrown by the invoked method
      Throwable cause = e.getCause();
      System.err.println("The underlying exception was: " + cause);
      e.printStackTrace();
    }
    Logger.recordOutput("Logger/closestTagID", closestTag);
    // If no valid tag was found, return a default value (e.g., 0)
    return closestTag == -1 ? 0 : closestTag - offset;
  }
}
