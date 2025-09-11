// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.Choreographer;

/** A minimal AutoFactory with three autos: Idle, Taxi, Taxi then Score L1 */
public class AutoFactory {
  private final DriverStation.Alliance alliance;
  private final RobotContainer robotContainer;

  // Tolerances for considering the robot "at" the target pose
  private static final double kPosTolMeters = 0.05; // 5 cm
  private static final double kRotTolRad = Math.toRadians(3.0); // 3 degrees

  public AutoFactory(final DriverStation.Alliance alliance, final RobotContainer robotContainer) {
    this.alliance = alliance;
    this.robotContainer = robotContainer;
  }

  private static final Command idleCommand = Commands.idle();

  // 1) Idle
  public Pair<Pose2d, Command> createIdleCommand() {
    return Pair.of(new Pose2d(), idleCommand);
  }

  // 2) Taxi (drive forward 2 meters from current pose)
  public Pair<Pose2d, Command> createTaxiCommand() {
    Pose2d initialPose = new Pose2d();
    Pose2d finalPose = forwardOffset(initialPose, 5);

    // Continuously command the drive to the goal and end when we reach it
    Command driveToPoint =
        Commands.run(
                () ->
                    robotContainer
                        .getDrive()
                        .setDesiredPoseForDriveToPointWithConstraints(finalPose, .5, .5),
                robotContainer.getDrive())
            .until(() -> atPose(finalPose, kPosTolMeters, kRotTolRad))
            .withName("Auto: Taxi 2m");

    return Pair.of(initialPose, driveToPoint);
  }

  // 3) Taxi then score L1 at the nearest "side" reef face to the starting side
  public Pair<Pose2d, Command> createTaxiThenScoreL1() {
    var taxi = createTaxiCommand();

    Command auto =
        Commands.sequence(
            taxi.getSecond(),
            robotContainer
                .getChoreographer()
                .setWantedChoreographyCommand(Choreographer.WantedChoreography.SCORE_L1));

    return Pair.of(taxi.getFirst(), auto);
  }

  // Helpers

  private Pose2d forwardOffset(Pose2d start, double meters) {
    Translation2d delta = new Translation2d(meters, 0.0).rotateBy(start.getRotation());
    return new Pose2d(start.getTranslation().plus(delta), start.getRotation());
  }

  // Checks if we’re within position and rotation tolerances of the goal
  private boolean atPose(Pose2d goal, double posTolMeters, double rotTolRad) {
    var current = RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry();
    double posErr = current.getTranslation().getDistance(goal.getTranslation());
    double rotErr = Math.abs(current.getRotation().minus(goal.getRotation()).getRadians());
    return posErr <= posTolMeters && rotErr <= rotTolRad;
  }
}
