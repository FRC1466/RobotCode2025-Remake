// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static frc.robot.FieldConstants.fieldWidth;
import static frc.robot.FieldConstants.startingLineX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.AlgaeObjective;
import frc.robot.FieldConstants.CoralObjective;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.*;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import lombok.RequiredArgsConstructor;

/**
 * The AutoBuilder class is responsible for constructing and managing autonomous routines for the
 * robot. It provides methods to create various autonomous sequences including multi-piece scoring,
 * one-piece scoring, and a simple taxi movement.
 *
 * <p>The class uses logged tunable parameters to adjust timing values dynamically, and manages the
 * robot's drive train and superstructure systems to execute complex autonomous patterns.
 *
 * <p>Each autonomous routine handles robot positioning, coral piece collection, and scoring at
 * specified reef levels and positions.
 */
@RequiredArgsConstructor
public class AutoBuilder {
  private static final LoggedTunableNumber scoreCancelSecs =
      new LoggedTunableNumber("AutoBuilder/ScoreCancelSeconds", 6);
  private static final LoggedTunableNumber intakeTimeSecs =
      new LoggedTunableNumber("AutoBuilder/IntakeTimeSecs", 1);

  private final Drive drive;
  private final Superstructure superstructure;
  private final BooleanSupplier push;

  private final double pushSecs = 0.5;

  public Command DefaultAuto() {
    return Autonomous(ReefLevel.L4, ReefLevel.L4, ReefLevel.L4, ReefLevel.L2, 8, 10, 11, 0);
  }

  public Command AlgaeAutonomous() {
    return AlgaeAuto(
        new CoralObjective(7, ReefLevel.L4),
        new AlgaeObjective(3),
        new AlgaeObjective(2),
        new AlgaeObjective(1),
        new AlgaeObjective(0),
        false,
        false,
        false,
        false);
  }

  public Command Autonomous(
      ReefLevel height1,
      ReefLevel height2,
      ReefLevel height3,
      ReefLevel height4,
      int branchId1,
      int branchId2,
      int branchId3,
      int branchId4) {

    CoralObjective[] coralObjectives =
        new CoralObjective[] {
          new CoralObjective(branchId1, height1),
          new CoralObjective(branchId2, height2),
          new CoralObjective(branchId3, height3),
          new CoralObjective(branchId4, height4)
        };
    Container<Integer> currentObjectiveIndex = new Container<>();

    var driveToStation = new DriveToStation(drive, true);
    Timer autoTimer = new Timer();
    Timer intakeTimer = new Timer();
    Timer coralIndexedTimer = new Timer();
    return Commands.runOnce(
            () -> {
              drive.setPose(
                  AllianceFlipUtil.apply(
                      MirrorUtil.apply(
                          new Pose2d(
                              startingLineX - Drive.robotWidth / 2.0,
                              fieldWidth - FieldConstants.Barge.closeCage.getY(),
                              Rotation2d.kPi))));
              superstructure.setAutoStart();
              autoTimer.restart();
              currentObjectiveIndex.value = 0;
            })
        .andThen(
            Push(),
            Commands.sequence(
                    // Intake
                    driveToStation
                        .deadlineFor(IntakeCommands.intake(superstructure))
                        .until(
                            () -> {
                              if (!driveToStation.withinTolerance(
                                  Units.inchesToMeters(5.0), Rotation2d.fromDegrees(5.0))) {
                                intakeTimer.restart();
                              }
                              return superstructure.hasCoral()
                                  || intakeTimer.hasElapsed(intakeTimeSecs.get());
                            }),
                    // Score
                    AutoScoreCommands.autoScore(
                            drive,
                            superstructure,
                            () -> coralObjectives[currentObjectiveIndex.value].reefLevel(),
                            () ->
                                Optional.of(
                                    MirrorUtil.apply(coralObjectives[currentObjectiveIndex.value])))
                        .finallyDo(
                            interrupted -> {
                              if (!interrupted) {
                                System.out.printf(
                                    "Scored Coral #"
                                        + (currentObjectiveIndex.value + 1)
                                        + " at %.2f\n",
                                    autoTimer.get());
                                currentObjectiveIndex.value++;
                              }
                            })
                        .beforeStarting(coralIndexedTimer::restart)
                        .raceWith(
                            Commands.waitUntil(
                                    () -> coralIndexedTimer.hasElapsed(scoreCancelSecs.get()))
                                .andThen(Commands.idle().onlyIf(superstructure::hasCoral))))
                .repeatedly()
                .until(() -> currentObjectiveIndex.value >= coralObjectives.length))
        .andThen(
            new DriveToPose(
                    drive,
                    () ->
                        drive
                            .getPose()
                            .transformBy(
                                GeomUtil.toTransform2d(
                                    -AutoScoreCommands.minDistanceReefClearL4.get(), 0.0)))
                .until(
                    () ->
                        AutoScoreCommands.outOfDistanceToReef(
                            drive.getPose(), AutoScoreCommands.minDistanceReefClearL4.get())));
  }

  public Command AlgaeAuto(
      CoralObjective score,
      AlgaeObjective height1,
      AlgaeObjective height2,
      AlgaeObjective height3,
      AlgaeObjective height4,
      boolean shouldProcess1,
      boolean shouldProcess2,
      boolean shouldProcess3,
      boolean shouldProcess4) {

    AlgaeObjective[] AlgaeObjectives = new AlgaeObjective[] {height1, height2, height3, height4};
    Container<Integer> currentObjectiveIndex = new Container<>();

    Timer autoTimer = new Timer();
    return Commands.runOnce(
            () -> {
              drive.setPose(
                  AllianceFlipUtil.apply(
                      MirrorUtil.apply(
                          new Pose2d(
                              startingLineX - Drive.robotWidth / 2.0,
                              fieldWidth - FieldConstants.Barge.closeCage.getY(),
                              Rotation2d.kPi))));
              superstructure.setAutoStart();
              autoTimer.restart();
              currentObjectiveIndex.value = 0;
            })
        .andThen(
            Push(),
            // Score the initial coral
            AutoScoreCommands.autoScore(
                    drive,
                    superstructure,
                    () -> score.reefLevel(),
                    () -> Optional.of(MirrorUtil.apply(score)))
                .finallyDo(
                    interrupted -> {
                      if (!interrupted) {
                        System.out.printf("Scored initial coral at %.2f\n", autoTimer.get());
                      }
                    }),
            Commands.sequence(
                    // Intake
                    AutoScoreCommands.reefIntake(
                            drive,
                            superstructure,
                            () ->
                                Optional.of(
                                    MirrorUtil.apply(AlgaeObjectives[currentObjectiveIndex.value])),
                            () -> 0,
                            () -> 0,
                            () -> 0,
                            Commands.none(),
                            () -> false,
                            () -> false,
                            false)
                        .until(() -> superstructure.hasAlgae()),
                    // Score
                    Commands.either(
                            // If should process
                            Commands.sequence(
                                // Drive to closer safe Y position first
                                Commands.either(
                                    new DriveToPose(
                                            drive,
                                            () ->
                                                new Pose2d(
                                                    drive.getPose().getX(),
                                                    1.75,
                                                    drive.getPose().getRotation()))
                                        .until(() -> Math.abs(drive.getPose().getY() - 1.75) < 0.1),
                                    new DriveToPose(
                                            drive,
                                            () ->
                                                new Pose2d(
                                                    drive.getPose().getX(),
                                                    6.5,
                                                    drive.getPose().getRotation()))
                                        .until(() -> Math.abs(drive.getPose().getY() - 6.5) < 0.1),
                                    () ->
                                        Math.abs(drive.getPose().getY() - 1.75)
                                            <= Math.abs(drive.getPose().getY() - 6.5)),
                                // Drive to safe X position
                                Commands.either(
                                    new DriveToPose(
                                            drive,
                                            () ->
                                                new Pose2d(
                                                    5.9,
                                                    drive.getPose().getY(),
                                                    drive.getPose().getRotation()))
                                        .until(() -> drive.getPose().getX() >= 5.9),
                                    Commands.none(),
                                    () -> drive.getPose().getX() < 5.9),
                                // TODO: Add processor scoring command here
                                Commands.none()),
                            // If not should process - barge scoring
                            Commands.sequence(
                                // Drive to closer safe Y position first
                                Commands.either(
                                    new DriveToPose(
                                            drive,
                                            () ->
                                                new Pose2d(
                                                    drive.getPose().getX(),
                                                    2.25,
                                                    drive.getPose().getRotation()))
                                        .until(() -> Math.abs(drive.getPose().getY() - 2.25) < 0.1),
                                    new DriveToPose(
                                            drive,
                                            () ->
                                                new Pose2d(
                                                    drive.getPose().getX(),
                                                    5.65,
                                                    drive.getPose().getRotation()))
                                        .until(() -> Math.abs(drive.getPose().getY() - 5.65) < 0.1),
                                    () ->
                                        Math.abs(drive.getPose().getY() - 2.25)
                                            <= Math.abs(drive.getPose().getY() - 5.65)),
                                // Drive to safe X position
                                Commands.either(
                                    new DriveToPose(
                                            drive,
                                            () ->
                                                new Pose2d(
                                                    7,
                                                    drive.getPose().getY(),
                                                    drive.getPose().getRotation()))
                                        .until(() -> drive.getPose().getX() >= 6.5),
                                    Commands.none(),
                                    () -> drive.getPose().getX() < 5.9),
                                AlgaeScoreCommands.netThrowLineup(
                                        drive,
                                        superstructure,
                                        () -> 0,
                                        () -> 0,
                                        Commands.none(),
                                        () -> false)
                                    .until(
                                        () -> {
                                          double currentX = drive.getPose().getX();
                                          double targetX =
                                              AllianceFlipUtil.applyX(
                                                  FieldConstants.fieldLength / 2.0 - 1.5);
                                          double currentY = drive.getPose().getY();
                                          double targetY =
                                              MathUtil.clamp(
                                                  drive.getPose().getY(),
                                                  AllianceFlipUtil.shouldFlip()
                                                      ? FieldConstants.fieldWidth
                                                          - Drive.robotWidth / 2.0
                                                      : FieldConstants.fieldWidth / 2.0
                                                          + Drive.robotWidth / 2.0,
                                                  AllianceFlipUtil.shouldFlip()
                                                      ? FieldConstants.fieldWidth / 2.0
                                                          + Drive.robotWidth / 2.0
                                                      : FieldConstants.fieldWidth
                                                          - Drive.robotWidth / 2.0);
                                          double currentRotation =
                                              drive.getPose().getRotation().getDegrees();
                                          double targetRotation =
                                              AllianceFlipUtil.apply(Rotation2d.k180deg)
                                                  .getDegrees();

                                          return EqualsUtil.epsilonEquals(currentX, targetX, 0.1)
                                              && EqualsUtil.epsilonEquals(currentY, targetY, 0.1)
                                              && EqualsUtil.epsilonEquals(
                                                  Math.abs(
                                                      Rotation2d.fromDegrees(currentRotation)
                                                          .minus(
                                                              Rotation2d.fromDegrees(
                                                                  targetRotation))
                                                          .getDegrees()),
                                                  0.0,
                                                  1.0);
                                        }),
                                AlgaeScoreCommands.netThrowScore(drive, superstructure)
                                    .withTimeout(1)),
                            () -> {
                              boolean[] shouldProcess = {
                                shouldProcess1, shouldProcess2, shouldProcess3, shouldProcess4
                              };
                              return currentObjectiveIndex.value < shouldProcess.length
                                  && shouldProcess[currentObjectiveIndex.value];
                            })
                        .finallyDo(
                            interrupted -> {
                              if (!interrupted) {
                                boolean[] shouldProcess = {
                                  shouldProcess1, shouldProcess2, shouldProcess3, shouldProcess4
                                };
                                System.out.printf(
                                    "Scored Algae #"
                                        + (currentObjectiveIndex.value + 1)
                                        + " at %.2f in the %s\n",
                                    autoTimer.get(),
                                    shouldProcess[currentObjectiveIndex.value]
                                        ? "processor"
                                        : "barge");
                                currentObjectiveIndex.value++;
                              }
                            }))
                .repeatedly()
                .until(() -> currentObjectiveIndex.value >= AlgaeObjectives.length))
        .andThen(
            new DriveToPose(
                    drive,
                    () ->
                        drive
                            .getPose()
                            .transformBy(
                                GeomUtil.toTransform2d(
                                    -AutoScoreCommands.minDistanceReefClearL4.get(), 0.0)))
                .until(
                    () ->
                        AutoScoreCommands.outOfDistanceToReef(
                            drive.getPose(), AutoScoreCommands.minDistanceReefClearL4.get())));
  }

  public Command TheOnePiece() {
    final var objective = new CoralObjective(7, ReefLevel.L4);
    return Commands.runOnce(
            () -> {
              drive.setPose(
                  AllianceFlipUtil.apply(
                      MirrorUtil.apply(
                          new Pose2d(
                              startingLineX - Drive.robotWidth / 2.0,
                              fieldWidth / 2.0,
                              Rotation2d.kPi))));
              superstructure.setAutoStart();
            })
        .andThen(
            Push(),
            AutoScoreCommands.autoScore(
                drive,
                superstructure,
                objective::reefLevel,
                () -> Optional.of(MirrorUtil.apply(objective))),
            new DriveToPose(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            AutoScoreCommands.getCoralScorePose(objective)
                                .plus(new Transform2d(-0.5, 0.0, Rotation2d.kZero))))
                .withTimeout(3.0)
                .deadlineFor(
                    superstructure.runGoal(
                        Superstructure.getScoringState(objective.reefLevel(), false))));
  }

  public Command Taxi() {
    return Commands.runOnce(
            () ->
                drive.setPose(
                    AllianceFlipUtil.apply(
                        new Pose2d(drive.getPose().getTranslation(), Rotation2d.kPi))))
        .andThen(
            new DriveToPose(
                    drive,
                    () -> drive.getPose(),
                    () -> drive.getPose(),
                    () ->
                        new Translation2d((AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0) * -1.0, 0.0),
                    () -> 0.0)
                .withTimeout(0.6));
  }

  private Command Push() {
    return new DriveToPose(
            drive,
            () -> {
              Pose2d robot = drive.getPose();
              return new Pose2d(
                  robot
                      .getTranslation()
                      .plus(new Translation2d(AllianceFlipUtil.shouldFlip() ? 0.1 : -0.1, 0.0)),
                  robot.getRotation());
            })
        .withTimeout(pushSecs)
        .onlyIf(push);
  }
}
