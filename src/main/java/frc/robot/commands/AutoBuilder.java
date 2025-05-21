// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static frc.robot.FieldConstants.fieldWidth;
import static frc.robot.FieldConstants.startingLineX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.CoralObjective;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.*;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import lombok.RequiredArgsConstructor;

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

  public Command FasterAuto() {
    return Autonomous(ReefLevel.L4, ReefLevel.L4, ReefLevel.L4, ReefLevel.L2, 8, 12, 0, 1);
  }

  public Command DensityAuto() {
    return Autonomous(ReefLevel.L2, ReefLevel.L2, ReefLevel.L2, ReefLevel.L2, 8, 12, 0, 1);
  }

  public Command DensityAutoFast() {
    return Autonomous(ReefLevel.L2, ReefLevel.L2, ReefLevel.L2, ReefLevel.L2, 8, 10, 11, 0);
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
            getUpInThePush(),
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

  public Command upInTheSimplicityAuto() {
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
            getUpInThePush(),
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

  public Command upInTheInspirationalAuto() {
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

  private Command getUpInThePush() {
    return new DriveToPose(
            drive,
            () -> {
              Pose2d robot = drive.getPose();
              return new Pose2d(
                  robot
                      .getTranslation()
                      .plus(new Translation2d(AllianceFlipUtil.shouldFlip() ? -0.1 : 0.1, 0.0)),
                  robot.getRotation());
            })
        .withTimeout(pushSecs)
        .onlyIf(push);
  }
}
