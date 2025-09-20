// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.constants.ChoreographerConstants.ScoringDirection;
import frc.robot.constants.ChoreographerConstants.ScoringSide;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ReefConstants;
import frc.robot.constants.ReefConstants.ReefFaces;
import frc.robot.subsystems.Choreographer;

public class AutoFactory {
  private final DriverStation.Alliance alliance;
  private final RobotContainer robotContainer;

  public AutoFactory(final DriverStation.Alliance alliance, final RobotContainer robotContainer) {
    this.alliance = alliance;
    this.robotContainer = robotContainer;
  }

  private static final Command idleCommand = Commands.idle();

  public Pair<Pose2d, Command> createIdleCommand() {
    return Pair.of(FieldConstants.getLeftStartingPose(alliance), idleCommand);
  }

  public Pair<Pose2d, Command> createTaxiCommand() {
    Pose2d initialPose = RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry();
    var targetPose =
        FieldConstants.isBlueAlliance()
            ? initialPose.minus(new Pose2d(2.0, 0.0, new Rotation2d()))
            : initialPose.minus(new Pose2d(-2.0, 0.0, new Rotation2d()));
    Pose2d finalPose = new Pose2d(targetPose.getTranslation(), targetPose.getRotation());
    return Pair.of(initialPose, driveToPoint(finalPose, 3, 1));
  }

  public Pair<Pose2d, Command> createJKLAuto() {
    Pose2d initialPose = FieldConstants.getLeftStartingPose(alliance);
    return Pair.of(
        initialPose,
        Commands.sequence(
            robotContainer.getChoreographer().setScoringSideCommand(ScoringSide.RIGHT),
            driveToFace(
                ReefFaces.IJ,
                ScoringDirection.FRONT,
                robotContainer.getChoreographer().getScoringSide()),
            setState(Choreographer.WantedChoreography.SCORE_L4),
            Commands.waitUntil(
                () ->
                    !robotContainer.getIntake().hasCoralClaw()
                        && !robotContainer.getIntake().hasCoralSlapdown()),
            stationIntakeCommand(),
            Commands.waitUntil(hasCoralSlapdown()),
            driveToFace(
                ReefFaces.KL,
                ScoringDirection.BACK,
                robotContainer.getChoreographer().getScoringSide()),
            robotContainer.getChoreographer().setScoringSideCommand(ScoringSide.LEFT),
            setState(Choreographer.WantedChoreography.SCORE_L4),
            Commands.waitUntil(
                () ->
                    !robotContainer.getIntake().hasCoralClaw()
                        && !robotContainer.getIntake().hasCoralSlapdown()),
            stationIntakeCommand(),
            Commands.waitUntil(hasCoralSlapdown()),
            driveToFace(
                ReefFaces.KL,
                ScoringDirection.BACK,
                robotContainer.getChoreographer().getScoringSide()),
            robotContainer.getChoreographer().setScoringSideCommand(ScoringSide.RIGHT),
            setState(Choreographer.WantedChoreography.SCORE_L4),
            Commands.waitUntil(
                () ->
                    !robotContainer.getIntake().hasCoralClaw()
                        && !robotContainer.getIntake().hasCoralSlapdown())));
  }

  public Command driveToFace(ReefFaces face, ScoringDirection direction, ScoringSide side) {
    Pose2d targetPose =
        FieldConstants.getDesiredFarScoringPoseForCoral(
            ReefConstants.blueAllianceReefFacesToIds.get(face), side, direction);
    return driveToPoint(targetPose, 3, 2);
  }

  public Trigger hasCoralClaw() {
    return new Trigger(() -> robotContainer.getIntake().hasCoralClaw());
  }

  public Trigger hasCoralSlapdown() {
    return new Trigger(() -> robotContainer.getIntake().hasCoralSlapdown());
  }

  public Command stationIntakeCommand() {
    return Commands.sequence(
        setState(Choreographer.WantedChoreography.INTAKE_CORAL_FROM_GROUND),
        driveToPoint(FieldConstants.getLeftStationPickup(alliance)),
        Commands.waitUntil(() -> robotContainer.getIntake().hasCoralSlapdown()),
        setState(Choreographer.WantedChoreography.DEFAULT_STATE));
  }

  public Command driveToPoint(Pose2d targetPose) {
    return Commands.sequence(
        Commands.runOnce(
            () -> robotContainer.getDrive().setDesiredPoseForDriveToPoint(targetPose)));
  }

  public Command driveToPoint(
      Pose2d targetPose, double maxLinearSpeedMeters, double maxAngularSpeedMeters) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                robotContainer
                    .getDrive()
                    .setDesiredPoseForDriveToPointWithConstraints(
                        targetPose, maxLinearSpeedMeters, maxAngularSpeedMeters)));
  }

  public Command setState(Choreographer.WantedChoreography choreography) {
    return robotContainer.getChoreographer().setChoreographyCommand(choreography);
  }
}
