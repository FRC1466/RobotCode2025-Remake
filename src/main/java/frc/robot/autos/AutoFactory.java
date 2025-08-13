// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autos;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ReefConstants;
import frc.robot.constants.ReefConstants.ReefFaces;
import frc.robot.constants.SuperstructureConstants;
import frc.robot.constants.SuperstructureConstants.ScoringDirection;
import frc.robot.subsystems.Superstructure;

/** A factory for creating autonomous programs for a given {@link Auto} */
@SuppressWarnings("unused")
public class AutoFactory {
  private final DriverStation.Alliance alliance;

  private final RobotContainer robotContainer;

  private final Choreo.TrajectoryCache trajectoryCache;

  private final double DISTANCE_TO_MOVE_ARM_UP = Units.inchesToMeters(48.0);

  /**
   * Create a new <code>AutoFactory</code>.
   *
   * @param robotContainer The {@link RobotContainer}
   */
  public AutoFactory(final DriverStation.Alliance alliance, final RobotContainer robotContainer) {
    this.alliance = alliance;
    this.robotContainer = robotContainer;

    trajectoryCache = new Choreo.TrajectoryCache();
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
    Pose2d initialPose = RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry();
    var targetPose =
        FieldConstants.isBlueAlliance()
            ? initialPose.minus(new Pose2d(2, 0, new Rotation2d()))
            : initialPose.minus(new Pose2d(-2, 0, new Rotation2d()));
    Pose2d finalPose = new Pose2d(targetPose.getTranslation(), targetPose.getRotation());
    return Pair.of(initialPose, driveToPoint(finalPose, 1.0));
  }

  public Pair<Pose2d, Command> createEDCAuto() {
    var initialPose = FieldConstants.getRightStartingPose(alliance);
    return Pair.of(
        initialPose,
        Commands.sequence(
            superFollowThenScore(ReefFaces.EF, Superstructure.WantedSuperState.SCORE_LEFT_L4),
            Commands.waitSeconds(.1),
            followThenIntakeFromStation(
                FieldConstants.getRightStationPickup(), Units.feetToMeters(10)),
            superFollowThenScore(ReefFaces.CD, Superstructure.WantedSuperState.SCORE_RIGHT_L4),
            Commands.waitSeconds(.1),
            followThenIntakeFromStation(
                FieldConstants.getRightStationPickup(), Units.feetToMeters(10)),
            superFollowThenScore(ReefFaces.CD, Superstructure.WantedSuperState.SCORE_LEFT_L4),
            Commands.waitSeconds(.1),
            followThenIntakeFromStation(
                FieldConstants.getRightStationPickup(), Units.feetToMeters(10))));
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
            followThenScore(
                ReefConstants.ReefFaces.IJ,
                Superstructure.WantedSuperState.SCORE_LEFT_L4,
                Units.inchesToMeters(72.0),
                Units.feetToMeters(8.0)),
            followThenIntakeFromStation(
                FieldConstants.getLeftStationPickup(alliance), Units.feetToMeters(10.0)),
            followThenScore(
                ReefConstants.ReefFaces.KL,
                Superstructure.WantedSuperState.SCORE_LEFT_L4,
                Units.inchesToMeters(72.0),
                Units.feetToMeters(10.0)),
            followThenIntakeFromStation(
                FieldConstants.getLeftStationPickup(alliance), Units.feetToMeters(12.0)),
            followThenScore(
                ReefConstants.ReefFaces.KL,
                Superstructure.WantedSuperState.SCORE_RIGHT_L4,
                Units.inchesToMeters(72.0),
                Units.feetToMeters(10.0)),
            followThenIntakeFromStation(
                FieldConstants.getLeftStationPickup(alliance), Units.feetToMeters(12.0)),
            driveToPoint(
                    FieldConstants.getDesiredPointToDriveToForCoralScoring(
                        offsetID,
                        SuperstructureConstants.ScoringSide.RIGHT,
                        SuperstructureConstants.ScoringDirection.LEFT,
                        Units.inchesToMeters(30.0)),
                    15.0)
                .until(
                    () ->
                        robotContainer.getDrive().getDistanceFromDriveToPointSetpoint()
                            < Units.inchesToMeters(80.0)),
            followThenScore(
                ReefConstants.ReefFaces.IJ,
                Superstructure.WantedSuperState.SCORE_RIGHT_L4,
                Units.inchesToMeters(60.0),
                Units.feetToMeters(10.0)),
            setState(Superstructure.WantedSuperState.DEFAULT_STATE)));
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
            followThenScore(
                ReefConstants.ReefFaces.EF,
                Superstructure.WantedSuperState.SCORE_RIGHT_L4,
                Units.inchesToMeters(72.0),
                Units.feetToMeters(8.0)),
            followThenIntakeFromStation(
                FieldConstants.getRightStationPickup(alliance), Units.feetToMeters(10.0)),
            followThenScore(
                ReefConstants.ReefFaces.CD,
                Superstructure.WantedSuperState.SCORE_RIGHT_L4,
                Units.inchesToMeters(72.0),
                Units.feetToMeters(10.0)),
            followThenIntakeFromStation(
                FieldConstants.getRightStationPickup(alliance), Units.feetToMeters(12.0)),
            followThenScore(
                ReefConstants.ReefFaces.CD,
                Superstructure.WantedSuperState.SCORE_LEFT_L4,
                Units.inchesToMeters(72.0),
                Units.feetToMeters(10.0)),
            followThenIntakeFromStation(
                FieldConstants.getRightStationPickup(alliance), Units.feetToMeters(12.0)),
            driveToPoint(
                    FieldConstants.getDesiredPointToDriveToForCoralScoring(
                        offsetID,
                        SuperstructureConstants.ScoringSide.LEFT,
                        ScoringDirection.LEFT,
                        Units.inchesToMeters(30.0)),
                    15.0)
                .until(
                    () ->
                        robotContainer.getDrive().getDistanceFromDriveToPointSetpoint()
                            < Units.inchesToMeters(80.0)),
            followThenScore(
                ReefConstants.ReefFaces.EF,
                Superstructure.WantedSuperState.SCORE_LEFT_L4,
                Units.inchesToMeters(60.0),
                Units.feetToMeters(10.0)),
            setState(Superstructure.WantedSuperState.DEFAULT_STATE)));
  }

  Command setState(Superstructure.WantedSuperState state) {
    return robotContainer.getSuperstructure().setStateCommand(state);
  }

  Command followTrajectory(Trajectory<SwerveSample> trajectory) {
    return new InstantCommand(
        () -> robotContainer.getDrive().setDesiredChoreoTrajectory(trajectory));
  }

  Command driveToPoint(Pose2d point, double maxVelocityOutputForDriveToPoint) {
    return new InstantCommand(
            () ->
                robotContainer
                    .getDrive()
                    .setDesiredPoseForDriveToPointWithConstraints(
                        point, maxVelocityOutputForDriveToPoint, 1.0))
        .andThen(new WaitUntilCommand(() -> robotContainer.getDrive().isAtDriveToPointSetpoint()));
  }

  Command driveToPointWithUnconstrainedMaxVelocity(
      Pose2d point, double maxVelocityOutputForDriveToPoint) {
    return new InstantCommand(
            () ->
                robotContainer
                    .getDrive()
                    .setDesiredPoseForDriveToPointWithConstraints(
                        point, maxVelocityOutputForDriveToPoint, Double.NaN))
        .andThen(new WaitUntilCommand(() -> robotContainer.getDrive().isAtDriveToPointSetpoint()));
  }

  private Command followThenScore(
      ReefConstants.ReefFaces reefFaces,
      Superstructure.WantedSuperState scoreState,
      double distanceFromEndOfPathtoMoveArmUp,
      double maxVelocity) {
    var desiredPose = getAutoScoringPose(reefFaces, scoreState);
    return ((driveToPoint(desiredPose, maxVelocity)
            .alongWith(
                new WaitUntilCommand(
                        () ->
                            robotContainer.getDrive().getDistanceFromDriveToPointSetpoint()
                                < distanceFromEndOfPathtoMoveArmUp)
                    .andThen(setState(scoreState)))))
        .andThen(waitForCoralRelease().deadlineFor(new WaitCommand(1.0)));
  }

  private Command followThenScore(
      ReefConstants.ReefFaces reefFaces,
      Superstructure.WantedSuperState scoreState,
      double distanceFromEndOfPathtoMoveArmUp) {
    var desiredPose = getAutoScoringPose(reefFaces, scoreState);
    return ((driveToPoint(desiredPose, Units.feetToMeters(8.0))
            .alongWith(
                new WaitUntilCommand(
                        () ->
                            robotContainer.getDrive().getDistanceFromDriveToPointSetpoint()
                                < distanceFromEndOfPathtoMoveArmUp)
                    .andThen(setState(scoreState)))))
        .andThen(waitForCoralRelease().raceWith(new WaitCommand(1.0)));
  }

  private Command followThenScoreWithNonDefaultMaxVelocity(
      ReefConstants.ReefFaces reefFaces,
      Superstructure.WantedSuperState scoreState,
      double maxVelocity) {
    var desiredPose = getAutoScoringPose(reefFaces, scoreState);
    return ((driveToPoint(desiredPose, maxVelocity).alongWith(setState(scoreState))))
        .andThen(waitForCoralRelease().raceWith(new WaitCommand(1.0)));
  }

  private Command followThenScoreWithMinimumReleaseTime(
      ReefConstants.ReefFaces reefFaces, Superstructure.WantedSuperState scoreState) {
    var desiredPose = getAutoScoringPose(reefFaces, scoreState);
    return ((driveToPoint(desiredPose, Units.feetToMeters(12.0)).alongWith(setState(scoreState))))
        .andThen(
            new WaitCommand(0.5).andThen(waitForCoralRelease()).raceWith(new WaitCommand(1.0)));
  }

  private Command followThenScore(
      ReefConstants.ReefFaces reefFaces, Superstructure.WantedSuperState scoreState) {
    var desiredPose = getAutoScoringPose(reefFaces, scoreState);
    return ((driveToPoint(desiredPose, Units.feetToMeters(12.0)).alongWith(setState(scoreState))))
        .andThen(waitForCoralRelease().raceWith(new WaitCommand(1.0)));
  }

  private Command superFollowThenScore(
      ReefConstants.ReefFaces reefFaces, Superstructure.WantedSuperState scoreState) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                robotContainer
                    .getSuperstructure()
                    .setHasDriveReachedIntermediatePoseForCoralScore(false)),
        (driveToAutoScoringPose(reefFaces, scoreState)
                .until(() -> robotContainer.getSuperstructure().isReadyToEjectInAutoPeriod())
                .alongWith(setState(scoreState)))
            .andThen(waitForCoralRelease().raceWith(new WaitCommand(1.0))));
  }

  private Command followThenScore(
      ReefConstants.ReefFaces reefFaces,
      Trajectory<SwerveSample> path,
      Superstructure.WantedSuperState scoreState) {
    return (followTrajectory(path)
            .andThen(
                new WaitUntilCommand(
                    robotContainer.getDrive()::isAtEndOfChoreoTrajectoryOrDriveToPoint)))
        .alongWith(
            new WaitUntilCommand(
                    () ->
                        robotContainer.getDrive().getRobotDistanceFromChoreoEndpoint()
                            < DISTANCE_TO_MOVE_ARM_UP)
                .andThen(followThenScore(reefFaces, scoreState)));
  }

  public Pose2d getIntakePose(Translation2d intakeLocation) {
    var angle =
        intakeLocation
            .minus(RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry().getTranslation())
            .getAngle();
    // var translation = intakeLocation.plus(new Translation2d(2.0, angle));
    return new Pose2d(intakeLocation, angle);
  }

  private Command followThenIntakeFromStation(Pose2d intakePose, double intakeVelocity) {
    return driveToPoint(intakePose, intakeVelocity)
        .alongWith(setState(Superstructure.WantedSuperState.INTAKE_CORAL_FROM_STATION))
        .andThen(
            Commands.waitUntil(() -> robotContainer.getSuperstructure().isReadyToIntakeCountdown()))
        .andThen(waitForCoralPickup().raceWith(Commands.waitSeconds(2.0)));
  }

  private Command waitForCoralRelease() {
    return new WaitUntilCommand(() -> !robotContainer.getIntake().hasCoralClaw());
  }

  private Command waitForCoralPickup() {
    return new WaitUntilCommand(() -> robotContainer.getIntake().hasCoralSlapdown());
  }

  public Pose2d getAutoScoringPose(
      ReefConstants.ReefFaces reefFaces, Superstructure.WantedSuperState superState) {
    var map =
        alliance == DriverStation.Alliance.Blue
            ? ReefConstants.blueAllianceReefFacesToIds
            : ReefConstants.redAllianceReefFacesToIds;
    var id = map.get(reefFaces);
    var scoringSide =
        (superState == Superstructure.WantedSuperState.SCORE_LEFT_L2
                || superState == Superstructure.WantedSuperState.SCORE_LEFT_L3
                || superState == Superstructure.WantedSuperState.SCORE_LEFT_L4)
            ? SuperstructureConstants.ScoringSide.LEFT
            : SuperstructureConstants.ScoringSide.RIGHT;
    return FieldConstants.getDesiredFinalScoringPoseForCoral(
        id, scoringSide, ScoringDirection.LEFT);
  }

  public Command driveToAutoScoringPose(
      ReefConstants.ReefFaces reefFaces, Superstructure.WantedSuperState superState) {
    var scoringSide =
        (superState == Superstructure.WantedSuperState.SCORE_LEFT_L2
                || superState == Superstructure.WantedSuperState.SCORE_LEFT_L3
                || superState == Superstructure.WantedSuperState.SCORE_LEFT_L4)
            ? SuperstructureConstants.ScoringSide.LEFT
            : SuperstructureConstants.ScoringSide.RIGHT;
    return Commands.run(
        () -> robotContainer.getSuperstructure().driveToScoringPose(reefFaces, scoringSide, false));
  }
}
