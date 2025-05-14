// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static frc.robot.commands.DriveCommands.DEADBAND;
import static frc.robot.commands.DriveToStation.withinDistanceToReef;
import static frc.robot.util.EqualsUtil.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.PathfindConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToScore extends DriveToPose {
  private static final LoggedTunableNumber ApproachEpsilon =
      new LoggedTunableNumber("DriveToScore/ApproachEpsilon", 0.1);
  private static final LoggedTunableNumber AngularAproachEpsilon =
      new LoggedTunableNumber("DriveToScore/AngularAproachEpsilon", 0.1);
  private static LoggedTunableNumber approachDistance =
      new LoggedTunableNumber("DriveToScore/ApproachDistance", .8);

  /** Constructor for teleop/manual control. Uses default zeroed suppliers for driver input. */
  public DriveToScore(Drive drive, boolean isAuto, Supplier<Integer> coralPovSupplier) {
    this(drive, () -> 0.0, () -> 0.0, () -> 0.0, isAuto, coralPovSupplier);
  }

  /**
   * Main constructor for DriveToScore. Accepts suppliers for driver X/Y translation, omega
   * (rotation), auto mode, and branch side supplier. Applies alliance flip to joystick input if
   * needed so consistent on all alliances.
   */
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
                .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
        () ->
            Math.copySign(
                Math.pow(MathUtil.applyDeadband(driverOmega.getAsDouble(), DEADBAND), 2.0),
                driverOmega.getAsDouble()),
        isAuto,
        () -> {
          int pov = coralPovSupplier.get();
          if (AllianceFlipUtil.shouldFlip()) {
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
          int tag = drive.getClosestTag();
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
          Logger.recordOutput("DriveToScore/ClosestTag", tag);

          if ((withinDistanceToReef(robot, currentApproachDistance)
                  || epsilonEquals(distance, 0, currentApproachEpsilon))
              && epsilonEquals(angularDistance, 0, currentAngularApproachEpsilon)) {
            return target;
          }
          return approach;
        },
        drive::getPose,
        linearFF,
        theta);
  }
}
