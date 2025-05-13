// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import java.util.List;
import java.util.Optional;

public class PathPlannerUtils {
  private final Drive driveSubsystem;

  /**
   * Creates a new PathPlannerManager.
   *
   * @param driveSubsystem The swerve drive subsystem to control
   */
  public PathPlannerUtils(Drive driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
  }

  public static final PathConstraints PATHFINDING_CONSTRAINTS =
      new PathConstraints(5.0, 3.5, Units.degreesToRadians(540), Units.degreesToRadians(460));

  /**
   * Creates a pathfinding command to the specified pose.
   *
   * @param target Target pose
   * @param velocity Target velocity
   * @return Pathfinding command
   */
  public Command getPathFinderCommand(Pose2d target, LinearVelocity velocity) {
    return AutoBuilder.pathfindToPose(target, PATHFINDING_CONSTRAINTS, velocity);
  }

  /**
   * @param pathName The path name to follow after pathfinding to it
   * @return The command to be scheduled
   */
  public Command getPathfindThenFollowPathCommand(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      Command pathCommand = AutoBuilder.pathfindThenFollowPath(path, PATHFINDING_CONSTRAINTS);
      return pathCommand;
    } catch (Exception e) {
      String errorMessage = "Failed to load path: " + pathName;
      return new PrintCommand(errorMessage + " Not following path!");
    }
  }

  /**
   * Gets the starting pose from a PathPlanner auto.
   *
   * @param autoName The name of the auto to load
   * @return Optional containing the starting pose, or empty if path cannot be loaded
   */
  public Optional<Pose2d> getAutoStartPose(String autoName) {
    try {
      List<PathPlannerPath> auto = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
      // Check to see if we have a traj
      if (auto.isEmpty()) return Optional.empty();
      // Load it
      PathPlannerPath path = auto.get(0);
      Optional<PathPlannerTrajectory> expectedTrajectory = path.getIdealTrajectory(Drive.PP_CONFIG);
      if (expectedTrajectory.isPresent()) {
        PathPlannerTrajectory trajectory = expectedTrajectory.get();
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
          return Optional.of(FlippingUtil.flipFieldPose(trajectory.getInitialState().pose));
        }
        return Optional.of(trajectory.getInitialState().pose);
      }
      return Optional.empty();
    } catch (Exception e) {
      System.err.println("Failed to load auto start pose: " + autoName);
      e.printStackTrace();
      return Optional.empty();
    }
  }

  /**
   * Gets the starting pose from a PathPlanner path.
   *
   * @param pathName The name of the path to load
   * @return Optional containing the starting pose, or empty if path cannot be loaded
   */
  public Optional<Pose2d> getPathStartPose(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      Optional<PathPlannerTrajectory> expectedTrajectory = path.getIdealTrajectory(Drive.PP_CONFIG);
      if (expectedTrajectory.isPresent()) {
        PathPlannerTrajectory trajectory = expectedTrajectory.get();
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
          return Optional.of(FlippingUtil.flipFieldPose(trajectory.getInitialState().pose));
        }
        return Optional.of(trajectory.getInitialState().pose);
      }
      return Optional.empty();
    } catch (Exception e) {
      System.err.println("Failed to load path start pose: " + pathName);
      e.printStackTrace();
      return Optional.empty();
    }
  }

  /**
   * Gets the final target pose from a PathPlanner path.
   *
   * @param pathName The name of the path to load
   * @return Optional containing the final pose, or empty if path cannot be loaded
   */
  public Optional<Pose2d> getPathEndPose(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      Optional<PathPlannerTrajectory> expectedTrajectory = path.getIdealTrajectory(Drive.PP_CONFIG);
      if (expectedTrajectory.isPresent()) {
        PathPlannerTrajectory trajectory = expectedTrajectory.get();
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
          return Optional.of(FlippingUtil.flipFieldPose(trajectory.getEndState().pose));
        }
        return Optional.of(trajectory.getEndState().pose);
      }
      return Optional.empty();
    } catch (Exception e) {
      System.err.println("Failed to load path end pose: " + pathName);
      e.printStackTrace();
      return Optional.empty();
    }
  }

  /**
   * Whether the current robot pose is near the end of a path
   *
   * @param pathName The path name to get the end pose from
   * @param translationalTolerance The translational tolerance in meters
   * @param rotationalTolerance The rotational tolerance in radians
   * @return true if near the end of a path
   */
  public boolean nearEndOfPath(
      String pathName, double translationalTolerance, double rotationalTolerance) {
    if (getPathEndPose(pathName).isPresent()) {
      Pose2d current = driveSubsystem.getPose();
      Pose2d target = getPathEndPose(pathName).get();

      // Check translational distance using hypot (handles X and Y together)
      double translationalDiff = current.getTranslation().getDistance(target.getTranslation());

      // Get absolute rotation difference and normalize to [-pi, pi]
      double rotationalDiff = current.getRotation().minus(target.getRotation()).getRadians();

      return translationalDiff <= translationalTolerance && rotationalDiff <= rotationalTolerance;
    }
    return false;
  }

  /**
   * Whether the current robot pose is near the start of a path
   *
   * @param pathName The path name to get the end pose from
   * @param translationalTolerance The translational tolerance in meters
   * @param rotationalTolerance The rotational tolerance in radians
   * @return true if near the start of a path
   */
  public boolean nearStartOfPath(
      String pathName, double translationalTolerance, double rotationalTolerance) {
    if (getPathStartPose(pathName).isPresent()) {
      Pose2d current = driveSubsystem.getPose();
      Pose2d target = getPathStartPose(pathName).get();

      // Check translational distance using hypot (handles X and Y together)
      double translationalDiff = current.getTranslation().getDistance(target.getTranslation());

      // Get absolute rotation difference and normalize to [-pi, pi]
      double rotationalDiff =
          Math.abs(current.getRotation().minus(target.getRotation()).getRadians());
      if (rotationalDiff > Math.PI) {
        rotationalDiff = 2 * Math.PI - rotationalDiff;
      }

      return translationalDiff <= translationalTolerance && rotationalDiff <= rotationalTolerance;
    }
    return false;
  }

  /**
   * Gets the trajectory points for a given path name.
   *
   * @param pathName The name of the path to get the trajectory for
   * @return Optional containing List of Pose2d points if path exists, empty Optional otherwise
   */
  public Optional<List<Pose2d>> getPathTrajectory(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      if (path == null) {
        return Optional.empty();
      }

      // Get the states at fixed intervals along the path
      List<Pose2d> trajectoryPoses = path.getPathPoses();
      return Optional.of(trajectoryPoses);
    } catch (Exception e) {
      return Optional.empty();
    }
  }
}
