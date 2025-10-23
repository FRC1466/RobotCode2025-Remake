// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveToPose;
import frc.robot.constants.ChoreographerConstants;
import frc.robot.constants.ChoreographerConstants.ScoringSide;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ReefConstants;
import frc.robot.constants.ReefConstants.ReefFaces;
import frc.robot.subsystems.Choreographer;
import java.io.IOException;
import org.json.simple.parser.ParseException;

/** A factory for creating autonomous programs for a given {@link Auto} */
@SuppressWarnings("unused")
public class AutoFactory {
  private final DriverStation.Alliance alliance;

  private final RobotContainer robotContainer;

  private final double DISTANCE_TO_MOVE_ARM_UP = Units.inchesToMeters(48.0);

  /**
   * Create a new <code>AutoFactory</code>.
   *
   * @param robotContainer The {@link RobotContainer}
   */
  public AutoFactory(final DriverStation.Alliance alliance, final RobotContainer robotContainer) {
    this.alliance = alliance;
    this.robotContainer = robotContainer;
  }

  /* Autonomous program factories
   *
   * Factory methods should be added here for each autonomous program.
   * The factory methods must:
   *   1. Be package-private (i.e. no access modifier)
   *   2. Accept no parameters
   *   3. Return a link Command
   */
  private static final Command idleCommand = Commands.idle();

  public Pair<Pose2d, Command> createIdleCommand() {
    return Pair.of(FieldConstants.getFarLeftStartingPose(alliance), idleCommand);
  }

  public Pair<Pose2d, Command> createTaxiCommand() {
    Pose2d initialPose = robotContainer.getDrive().getPose();
    var transform =
        FieldConstants.isBlueAlliance()
            ? new Transform2d(2, 0, new Rotation2d())
            : new Transform2d(-2, 0, new Rotation2d());
    Pose2d finalPose = initialPose.transformBy(transform);
    return Pair.of(initialPose, driveToPoint(finalPose));
  }

  public Pair<Pose2d, Command> createEDCAuto() {
    var initialPose = FieldConstants.getRightStartingPose(alliance);
    return Pair.of(
        initialPose,
        Commands.sequence(
            robotContainer.getChoreographer().setScoringSideCommand(ScoringSide.LEFT),
            superFollowThenScore(ReefFaces.EF, Choreographer.WantedChoreography.SCORE_L4),
            followThenIntakeFromStation(FieldConstants.getRightStationPickup(alliance)),
            robotContainer.getChoreographer().setScoringSideCommand(ScoringSide.RIGHT),
            superFollowThenScore(ReefFaces.CD, Choreographer.WantedChoreography.SCORE_L4),
            followThenIntakeFromStation(FieldConstants.getRightStationPickup(alliance)),
            robotContainer.getChoreographer().setScoringSideCommand(ScoringSide.LEFT),
            superFollowThenScore(ReefFaces.CD, Choreographer.WantedChoreography.SCORE_L4),
            followThenIntakeFromStation(FieldConstants.getRightStationPickup(alliance))));
  }

  public Pair<Pose2d, Command> createIKLJAuto() {
    var offsetID =
        alliance == DriverStation.Alliance.Blue
            ? ReefConstants.blueAllianceReefFacesToIds.get(ReefConstants.ReefFaces.IJ)
            : ReefConstants.redAllianceReefFacesToIds.get(ReefConstants.ReefFaces.IJ);
    var initialPose = FieldConstants.getLeftStartingPose(alliance);
    return Pair.of(
        initialPose,
        Commands.sequence(
            robotContainer.getChoreographer().setScoringSideCommand(ScoringSide.LEFT),
            followThenScore(ReefFaces.IJ, Choreographer.WantedChoreography.SCORE_L4),
            followThenIntakeFromStation(FieldConstants.getLeftStationPickup(alliance)),
            robotContainer.getChoreographer().setScoringSideCommand(ScoringSide.LEFT),
            followThenScore(ReefFaces.KL, Choreographer.WantedChoreography.SCORE_L4),
            followThenIntakeFromStation(FieldConstants.getLeftStationPickup(alliance)),
            robotContainer.getChoreographer().setScoringSideCommand(ScoringSide.RIGHT),
            followThenScore(ReefFaces.KL, Choreographer.WantedChoreography.SCORE_L4),
            followThenIntakeFromStation(FieldConstants.getLeftStationPickup(alliance)),
            driveToPoint(
                FieldConstants.getDesiredPointToDriveToForCoralScoring(
                    offsetID,
                    ChoreographerConstants.ScoringSide.RIGHT,
                    Units.inchesToMeters(30.0))),
            robotContainer.getChoreographer().setScoringSideCommand(ScoringSide.RIGHT),
            followThenScore(ReefFaces.IJ, Choreographer.WantedChoreography.SCORE_L4),
            setState(Choreographer.WantedChoreography.DEFAULT_STATE)));
  }

  public Pair<Pose2d, Command> createFDCEAuto() {
    var offsetID =
        alliance == DriverStation.Alliance.Blue
            ? ReefConstants.blueAllianceReefFacesToIds.get(ReefConstants.ReefFaces.EF)
            : ReefConstants.redAllianceReefFacesToIds.get(ReefConstants.ReefFaces.EF);
    var initialPose = FieldConstants.getRightStartingPose(alliance);
    return Pair.of(
        initialPose,
        Commands.sequence(
            robotContainer.getChoreographer().setScoringSideCommand(ScoringSide.RIGHT),
            followThenScore(ReefFaces.EF, Choreographer.WantedChoreography.SCORE_L4),
            followThenIntakeFromStation(FieldConstants.getRightStationPickup(alliance)),
            robotContainer.getChoreographer().setScoringSideCommand(ScoringSide.RIGHT),
            followThenScore(ReefFaces.CD, Choreographer.WantedChoreography.SCORE_L4),
            followThenIntakeFromStation(FieldConstants.getRightStationPickup(alliance)),
            robotContainer.getChoreographer().setScoringSideCommand(ScoringSide.LEFT),
            followThenScore(ReefFaces.CD, Choreographer.WantedChoreography.SCORE_L4),
            followThenIntakeFromStation(FieldConstants.getRightStationPickup(alliance)),
            driveToPoint(
                FieldConstants.getDesiredPointToDriveToForCoralScoring(
                    offsetID, ChoreographerConstants.ScoringSide.LEFT, Units.inchesToMeters(30.0))),
            robotContainer.getChoreographer().setScoringSideCommand(ScoringSide.LEFT),
            followThenScore(ReefFaces.EF, Choreographer.WantedChoreography.SCORE_L4),
            setState(Choreographer.WantedChoreography.DEFAULT_STATE)));
  }

  Command setState(Choreographer.WantedChoreography state) {
    return robotContainer.getChoreographer().setChoreographyCommand(state);
  }

  Command followPath(String pathName) {
    Command pathCommand = Commands.none();
    try {
      pathCommand = AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
    } catch (FileVersionException | IOException | ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    return pathCommand;
  }

  Command driveToPoint(Pose2d point) {
    return new DriveToPose(robotContainer.getDrive(), () -> point);
  }

  private Command followThenScore(
      ReefConstants.ReefFaces reefFaces,
      Choreographer.WantedChoreography scoreState,
      double distanceFromEndOfPathToMoveArmUp) {
    var desiredPose = getAutoScoringPose(reefFaces, scoreState);
    var driveCommand = driveToPoint(desiredPose);
    return driveCommand
        .alongWith(
            Commands.waitUntil(
                    () ->
                        robotContainer
                                .getDrive()
                                .getPose()
                                .getTranslation()
                                .getDistance(desiredPose.getTranslation())
                            < distanceFromEndOfPathToMoveArmUp)
                .andThen(setState(scoreState)))
        .andThen(waitForCoralRelease().deadlineWith(Commands.waitSeconds(1.0)));
  }

  private Command followThenScore(
      ReefConstants.ReefFaces reefFaces, Choreographer.WantedChoreography scoreState) {
    var desiredPose = getAutoScoringPose(reefFaces, scoreState);
    return driveToPoint(desiredPose)
        .alongWith(setState(scoreState))
        .andThen(waitForCoralRelease().raceWith(Commands.waitSeconds(1.0)));
  }

  private Command superFollowThenScore(
      ReefConstants.ReefFaces reefFaces, Choreographer.WantedChoreography scoreState) {
    return Commands.sequence(
        Commands.runOnce(() -> robotContainer.getChoreographer().resetDebouncers()),
        driveToAutoScoringPose(reefFaces, scoreState)
            .until(() -> robotContainer.getChoreographer().isReadyToEjectInAutoPeriod())
            .alongWith(setState(scoreState))
            .andThen(waitForCoralRelease().raceWith(Commands.waitSeconds(1.0))));
  }

  private Command followThenScore(
      ReefConstants.ReefFaces reefFaces,
      String pathName,
      Choreographer.WantedChoreography scoreState) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return followPath(pathName)
          .alongWith(
              Commands.waitUntil(
                      () ->
                          robotContainer
                                  .getDrive()
                                  .getPose()
                                  .getTranslation()
                                  .getDistance(
                                      path.getStartingHolonomicPose().get().getTranslation())
                              < DISTANCE_TO_MOVE_ARM_UP)
                  .andThen(followThenScore(reefFaces, scoreState)));
    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
      return followPath(pathName).andThen(followThenScore(reefFaces, scoreState));
    }
  }

  public Pose2d getIntakePose(Translation2d intakeLocation) {
    var angle =
        intakeLocation.minus(robotContainer.getDrive().getPose().getTranslation()).getAngle();
    return new Pose2d(intakeLocation, angle);
  }

  private Command followThenIntakeFromStation(Pose2d intakePose) {
    return driveToPoint(intakePose)
        .alongWith(setState(Choreographer.WantedChoreography.INTAKE_CORAL_FROM_STATION))
        .andThen(
            Commands.waitUntil(() -> robotContainer.getChoreographer().isReadyToIntakeCountdown()))
        .andThen(waitForCoralPickup().raceWith(Commands.waitSeconds(2.0)));
  }

  private Command waitForCoralRelease() {
    return Commands.waitUntil(() -> !robotContainer.getIntake().hasCoral());
  }

  private Command waitForCoralPickup() {
    return Commands.waitUntil(() -> robotContainer.getIntake().hasCoral());
  }

  public Pose2d getAutoScoringPose(
      ReefConstants.ReefFaces reefFaces, Choreographer.WantedChoreography superState) {
    var map =
        alliance == DriverStation.Alliance.Blue
            ? ReefConstants.blueAllianceReefFacesToIds
            : ReefConstants.redAllianceReefFacesToIds;
    var id = map.get(reefFaces);
    return FieldConstants.getDesiredFinalScoringPoseForCoral(
        id, robotContainer.getChoreographer().getScoringSide());
  }

  public Command driveToAutoScoringPose(
      ReefConstants.ReefFaces reefFaces, Choreographer.WantedChoreography superState) {
    return Commands.run(
        () ->
            robotContainer
                .getChoreographer()
                .driveToScoringPose(
                    reefFaces, robotContainer.getChoreographer().getScoringSide(), false));
  }
}
