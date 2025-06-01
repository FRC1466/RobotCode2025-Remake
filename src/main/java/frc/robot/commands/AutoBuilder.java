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
import frc.robot.FieldConstants.AlgaeObjective;
import frc.robot.FieldConstants.CoralObjective;
import frc.robot.FieldConstants.IceCreamObjective;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.*;
import frc.robot.util.PathTokenParser.Token;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

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
    return CoralAutonomous(ReefLevel.L4, ReefLevel.L4, ReefLevel.L4, ReefLevel.L2, 8, 10, 11, 0);
  }

  public Command CoralAutonomous(
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

  public Command SuperDuperSecretAuto(String strategy) {
    Logger.recordOutput("AutoLogger", "Starting SuperDuperSecretAuto with strategy: " + strategy);

    var parsedStrat = PathTokenParser.parse(strategy);
    Logger.recordOutput("AutoLogger", "Parsed " + parsedStrat.size() + " tokens from strategy");

    // Convert tokens to command sequence
    Command[] commands = new Command[parsedStrat.size()];
    for (int i = 0; i < parsedStrat.size(); i++) {
      Token token = parsedStrat.get(i);
      Logger.recordOutput(
          "AutoLogger",
          "Processing token "
              + i
              + ": type="
              + token.type
              + ", values="
              + java.util.Arrays.toString(token.values));

      switch (token.type) {
        case C:
          // C-(branchId)-(reefLevel): autoscore coral
          int branchId = Integer.parseInt(token.values[0]);
          ReefLevel level = ReefLevel.valueOf("L" + token.values[1]);
          CoralObjective objective = new CoralObjective(branchId, level);
          Logger.recordOutput(
              "AutoLogger",
              "Created coral score command for branch " + branchId + " at level " + level);
          commands[i] =
              AutoScoreCommands.autoScore(
                      drive,
                      superstructure,
                      () -> level,
                      () -> Optional.of(MirrorUtil.apply(objective)))
                  .beforeStarting(
                      () ->
                          Logger.recordOutput(
                              "AutoLogger",
                              "Starting coral score for branch " + branchId + " at level " + level))
                  .finallyDo(
                      interrupted ->
                          Logger.recordOutput(
                              "AutoLogger",
                              "Finished coral score for branch "
                                  + branchId
                                  + " - interrupted: "
                                  + interrupted));
          break;
        case S:
          Logger.recordOutput("AutoLogger", "Created drive to station command");
          commands[i] =
              new DriveToStation(drive, true)
                  .alongWith(IntakeCommands.intake(superstructure))
                  .until(() -> superstructure.hasCoral())
                  .beforeStarting(
                      () -> Logger.recordOutput("AutoLogger", "Starting drive to station"))
                  .finallyDo(
                      interrupted ->
                          Logger.recordOutput(
                              "AutoLogger",
                              "Finished drive to station - interrupted: " + interrupted));
          break;
        case A:
          // A-(level): algae reef intake
          int algaeLevel = Integer.parseInt(token.values[0]);
          Logger.recordOutput("AutoLogger", "Created algae intake command for level " + algaeLevel);
          commands[i] =
              AutoScoreCommands.reefIntake(
                      drive,
                      superstructure,
                      () -> Optional.of(new AlgaeObjective(algaeLevel)),
                      () -> 0.0,
                      () -> 0.0,
                      () -> 0.0,
                      Commands.none(),
                      () -> false,
                      () -> false,
                      false)
                  .until(() -> superstructure.hasAlgae())
                  .beforeStarting(
                      () ->
                          Logger.recordOutput(
                              "AutoLogger", "Starting algae intake at level " + algaeLevel))
                  .finallyDo(
                      interrupted ->
                          Logger.recordOutput(
                              "AutoLogger",
                              "Finished algae intake at level "
                                  + algaeLevel
                                  + " - interrupted: "
                                  + interrupted));
          break;
        case I:
          // I-(level): Ice Cream lollipop
          int iceCreamLevel = Integer.parseInt(token.values[0]);
          int iceCreamID = iceCreamLevel - 1;
          Logger.recordOutput(
              "AutoLogger",
              "Created ice cream command for level " + iceCreamLevel + " (ID: " + iceCreamID + ")");
          commands[i] =
              Commands.sequence(
                      AutoScoreCommands.iceCreamIntake(
                          drive,
                          superstructure,
                          () -> Optional.of(new IceCreamObjective(iceCreamID)),
                          () -> 0.0,
                          () -> 0.0,
                          () -> 0.0,
                          Commands.none(),
                          () -> false,
                          () -> false,
                          () -> false),
                      AutoScoreCommands.iceCreamIntake(
                          drive,
                          superstructure,
                          () -> Optional.of(new IceCreamObjective(iceCreamID)),
                          () -> 0.0,
                          () -> 0.0,
                          () -> 0.0,
                          Commands.none(),
                          () -> false,
                          () -> false,
                          () -> true))
                  .beforeStarting(
                      () ->
                          Logger.recordOutput(
                              "AutoLogger",
                              "Starting ice cream sequence for level " + iceCreamLevel))
                  .finallyDo(
                      interrupted ->
                          Logger.recordOutput(
                              "AutoLogger",
                              "Finished ice cream sequence for level "
                                  + iceCreamLevel
                                  + " - interrupted: "
                                  + interrupted));
          break;
        case P:
          // P-(O, S): algae process
          String location = token.values[0];
          Logger.recordOutput(
              "AutoLogger", "Created algae process command for location " + location);
          commands[i] =
              Commands.sequence(
                      AlgaeScoreCommands.process(
                          drive,
                          superstructure,
                          () -> 0.0, // driverX
                          () -> 0.0, // driverY
                          () -> 0.0, // driverOmega
                          // joystickDrive
                          () -> Commands.none(),
                          () -> location.equals("O"), // onOpposingSide
                          () -> true, // holdingButton
                          false, // eject
                          () -> false // disableAlgaeScoreAutoAlign
                          ),
                      AlgaeScoreCommands.process(
                          drive,
                          superstructure,
                          () -> 0.0, // driverX
                          () -> 0.0, // driverY
                          () -> 0.0, // driverOmega
                          // joystickDrive
                          () -> Commands.none(),
                          () -> location.equals("O"), // onOpposingSide
                          () -> true, // holdingButton
                          true, // eject
                          () -> false // disableAlgaeScoreAutoAlign
                          ))
                  .beforeStarting(
                      () ->
                          Logger.recordOutput(
                              "AutoLogger", "Starting algae process at location " + location))
                  .finallyDo(
                      interrupted ->
                          Logger.recordOutput(
                              "AutoLogger",
                              "Finished algae process at location "
                                  + location
                                  + " - interrupted: "
                                  + interrupted));
          break;
        case B:
          // B: algae barge align and throw
          Logger.recordOutput("AutoLogger", "Created barge align and throw command");
          commands[i] =
              Commands.sequence(
                      AlgaeScoreCommands.netThrowLineup(
                              drive,
                              superstructure,
                              () -> 0.0, // driverX
                              () -> 0.0, // driverY
                              Commands.none(), // joystickDrive
                              () -> false // disableAlgaeScoreAutoAlign
                              )
                          .until(
                              () ->
                                  EqualsUtil.epsilonEquals(
                                      drive.getPose().getX(),
                                      AllianceFlipUtil.applyX(
                                          FieldConstants.fieldLength / 2.0
                                              - AlgaeScoreCommands.throwLineupDistance.get()),
                                      .25)),
                      AlgaeScoreCommands.netThrowScore(drive, superstructure)
                          .deadlineFor(
                              Commands.waitUntil(() -> !superstructure.hasAlgae()),
                              Commands.waitSeconds(2.25)))
                  .beforeStarting(
                      () -> Logger.recordOutput("AutoLogger", "Starting barge align and throw"))
                  .finallyDo(
                      interrupted ->
                          Logger.recordOutput(
                              "AutoLogger",
                              "Finished barge align and throw - interrupted: " + interrupted));
          break;
        case POSE:
          // POSE-(x)-(y): drive to pose
          superstructure.runGoal(SuperstructureState.STOWTRAVEL);
          if (superstructure.hasAlgae()) {
            superstructure.runGoal(SuperstructureState.PRE_PROCESS);
          }
          double x = Double.parseDouble(token.values[0]);
          double y = Double.parseDouble(token.values[1]);
          double rot = Double.parseDouble(token.values[2]);
          // Normalize rotation from 0-360 to -180 to 180
          if (rot < -180 || rot > 180) {
            rot = ((rot + 180) % 360 + 360) % 360 - 180; // Normalize to -180 to 180
          }
          double finalRot = rot;
          Pose2d targetPose = new Pose2d(x, y, new Rotation2d(Units.degreesToRadians(rot)));
          Logger.recordOutput(
              "AutoLogger", "Created drive to pose command: (" + x + ", " + y + ", " + rot + "°)");
          commands[i] =
              new DriveToPose(drive, () -> targetPose)
                  .until(
                      () ->
                          EqualsUtil.epsilonEquals(drive.getPose().getX(), targetPose.getX(), 0.1)
                              && EqualsUtil.epsilonEquals(
                                  drive.getPose().getY(), targetPose.getY(), 0.1)
                              && EqualsUtil.epsilonEquals(
                                  drive.getPose().getRotation().getDegrees(),
                                  targetPose.getRotation().getDegrees(),
                                  10))
                  .beforeStarting(
                      () ->
                          Logger.recordOutput(
                              "AutoLogger",
                              "Starting drive to pose (" + x + ", " + y + ", " + finalRot + "°)"))
                  .finallyDo(
                      interrupted ->
                          Logger.recordOutput(
                              "AutoLogger",
                              "Finished drive to pose - interrupted: " + interrupted));
          break;
        default:
          Logger.recordOutput(
              "AutoLogger", "Unknown token type: " + token.type + ", using no-op command");
          commands[i] = Commands.none();
          break;
      }
    }

    Logger.recordOutput(
        "AutoLogger", "Created " + commands.length + " commands, starting sequence");

    // Start with initialization command
    Command autoCommand =
        Commands.runOnce(
            () -> {
              superstructure.setAutoStart();
              Logger.recordOutput("AutoLogger", "Auto start initialized");
            });

    // Chain each command sequentially using andThen
    for (int i = 0; i < commands.length; i++) {
      final int commandIndex = i;
      autoCommand =
          autoCommand.andThen(
              Commands.runOnce(
                      () -> Logger.recordOutput("AutoLogger", "Starting command " + commandIndex))
                  .andThen(commands[i])
                  .andThen(
                      Commands.runOnce(
                          () ->
                              Logger.recordOutput(
                                  "AutoLogger", "Completed command " + commandIndex))));
    }

    return autoCommand.finallyDo(
        interrupted ->
            Logger.recordOutput(
                "AutoLogger", "SuperDuperSecretAuto completed - interrupted: " + interrupted));
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
