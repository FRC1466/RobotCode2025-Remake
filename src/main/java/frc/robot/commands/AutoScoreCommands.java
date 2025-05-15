// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.FieldConstants.AlgaeObjective;
import frc.robot.FieldConstants.CoralObjective;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.*;
import java.util.Optional;
import java.util.function.*;
import org.littletonrobotics.junction.Logger;

public class AutoScoreCommands {
  // Radius of regular hexagon is side length
  private static final double reefRadius = Reef.faceLength;
  private static final double controllerRumbleSecs = 0.2;

  private static final LoggedTunableNumber maxDistanceReefLineupX =
      new LoggedTunableNumber("AutoScore/MaxDistanceReefLineupX", 1.0);
  private static final LoggedTunableNumber maxDistanceReefLineupY =
      new LoggedTunableNumber("AutoScore/MaxDistanceReefLineupY", 1.2);
  public static final LoggedTunableNumber minDistanceReefClearL4 =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClear", Units.inchesToMeters(4.0));
  public static final LoggedTunableNumber minDistanceReefClearAlgaeL4 =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClearAlgae", 0.45);
  public static final LoggedTunableNumber minAngleReefClear =
      new LoggedTunableNumber("AutoScore/MinAngleReefClear", 30.0);
  public static final LoggedTunableNumber algaeBackupTime =
      new LoggedTunableNumber("AutoScore/AlgaeBackupTime", 0.5);
  private static final LoggedTunableNumber distanceSuperstructureReady =
      new LoggedTunableNumber("AutoScore/DistanceSuperstructureReady", 2);
  private static final LoggedTunableNumber distanceSuperstructureReadyAuto =
      new LoggedTunableNumber("AutoScore/DistanceSuperstructureReadyAuto", 2.0);
  private static final LoggedTunableNumber thetaToleranceReady =
      new LoggedTunableNumber("AutoScore/ThetaToleranceReady", 35.0);
  private static final LoggedTunableNumber arcDistanceReady =
      new LoggedTunableNumber("AutoScore/ArcDistanceReady", 0.7);
  private static final LoggedTunableNumber arcDistanceReadyAuto =
      new LoggedTunableNumber("AutoScore/ArcDistanceReadyAuto", 1.5);
  private static final LoggedTunableNumber[] lowerXToleranceEject = {
    new LoggedTunableNumber("AutoScore/XToleranceEject/Lower/L1", 0.1),
    new LoggedTunableNumber("AutoScore/XToleranceEject/Lower/L2", 0.1),
    new LoggedTunableNumber("AutoScore/XToleranceEject/Lower/L3", 0.1),
    new LoggedTunableNumber("AutoScore/XToleranceEject/Lower/L4", 0.1)
  };
  private static final LoggedTunableNumber[] upperXToleranceEject = {
    new LoggedTunableNumber("AutoScore/XToleranceEject/Upper/L1", 0.1),
    new LoggedTunableNumber("AutoScore/XToleranceEject/Upper/L2", 0.1),
    new LoggedTunableNumber("AutoScore/XToleranceEject/Upper/L3", 0.1),
    new LoggedTunableNumber("AutoScore/XToleranceEject/Upper/L4", 0.1)
  };
  private static final LoggedTunableNumber[] yToleranceEject = {
    new LoggedTunableNumber("AutoScore/YToleranceEject/L1", 0.1),
    new LoggedTunableNumber("AutoScore/YToleranceEject/L2", 0.1),
    new LoggedTunableNumber("AutoScore/YToleranceEject/L3", 0.1),
    new LoggedTunableNumber("AutoScore/YToleranceEject/L4", 0.1)
  };
  private static final LoggedTunableNumber[] lookaheadEject = {
    new LoggedTunableNumber("AutoScore/LookaheadToleranceEject/L1", 0.5),
    new LoggedTunableNumber("AutoScore/LookaheadToleranceEject/L2", 0.5),
    new LoggedTunableNumber("AutoScore/LookaheadToleranceEject/L3", 0.5),
    new LoggedTunableNumber("AutoScore/LookaheadToleranceEject/L4", 0.3)
  };
  private static final LoggedTunableNumber l1ThetaEject =
      new LoggedTunableNumber("AutoScore/L1ThetaToleranceEject", 5.0);
  private static final LoggedTunableNumber l4EjectDelay =
      new LoggedTunableNumber("AutoScore/L4EjectDelay", 0.05);
  private static final LoggedTunableNumber l4EjectDelayAuto =
      new LoggedTunableNumber("AutoScore/L4EjectDelayAuto", 0.05);
  private static final LoggedTunableNumber l2ReefIntakeDistance =
      new LoggedTunableNumber("AutoScore/L2ReefIntakeDistance", 0.12);
  private static final LoggedTunableNumber l3ReefIntakeDistance =
      new LoggedTunableNumber("AutoScore/L3ReefIntakeDistance", 0.14);
  private static final LoggedTunableNumber l2ScoreDistance =
      new LoggedTunableNumber("AutoScore/L2ScoreDistance", 0.1);
  private static final LoggedTunableNumber l3ScoreDistance =
      new LoggedTunableNumber("AutoScore/L3ScoreDistance", 0.1);
  private static final LoggedTunableNumber l4ScoreDistance =
      new LoggedTunableNumber("AutoScore/L4ScoreDistance", 0.15);
  private static final LoggedTunableNumber maxAimingAngle =
      new LoggedTunableNumber("AutoScore/MaxAimingAngle", 20.0);
  private static final LoggedTunableNumber l1AlignOffsetX =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetX", 0.45);
  private static final LoggedTunableNumber l1AlignOffsetY =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetY", 0.0);
  private static final LoggedTunableNumber l1AlignOffsetTheta =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetTheta", 180.0);
  private static final LoggedTunableNumber[] ejectTimeSeconds = {
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L1", 0.5),
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L2", 0.5),
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L3", 0.5),
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L4", 0.3)
  };
  private static final LoggedTunableNumber[] branchFudgeX = {
    new LoggedTunableNumber("AutoScore/FudgeX/L1", 0.0),
    new LoggedTunableNumber("AutoScore/FudgeX/L2", 0.0),
    new LoggedTunableNumber("AutoScore/FudgeX/L3", 0.0),
    new LoggedTunableNumber("AutoScore/FudgeX/L4", 0.0)
  };
  private static final boolean enableAimingInAuto = true;

  private AutoScoreCommands() {}

  @SuppressWarnings("unused")
  public static Command autoScore(
      Drive drive,
      Superstructure superstructure,
      RollerSystem funnel,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Command joystickDrive,
      Command controllerRumble,
      BooleanSupplier robotRelative,
      BooleanSupplier disableReefAutoAlign,
      BooleanSupplier manualEject) {
    Supplier<Pose2d> robot = drive::getPose;
    Function<CoralObjective, Pose2d> goal =
        objective -> {
          boolean hasAlgae = superstructure.hasAlgae();
          if (reefLevel.get() == ReefLevel.L1) {
            return AllianceFlipUtil.apply(getL1Pose(objective));
          }
          Pose2d goalPose = getCoralScorePose(objective);
          final double clearDistance =
              hasAlgae ? minDistanceReefClearAlgaeL4.get() : minDistanceReefClearL4.get();
          if ((!superstructure.readyForL4() && reefLevel.get() == ReefLevel.L4)
              || (superstructure.readyForL4()
                  && withinDistanceToReef(robot.get(), clearDistance - 0.05)
                  && reefLevel.get() != ReefLevel.L4)) {
            goalPose =
                goalPose.transformBy(
                    GeomUtil.toTransform2d(
                        goalPose.getTranslation().getDistance(Reef.center)
                            - reefRadius
                            - (Drive.DRIVE_BASE_LENGTH / 2.0)
                            - clearDistance,
                        0.0));
          }
          Logger.recordOutput("AutoScore/Goal", AllianceFlipUtil.apply(goalPose));
          if (DriverStation.isAutonomousEnabled() && !enableAimingInAuto) {
            return AllianceFlipUtil.apply(goalPose);
          } else {
            Pose2d flippedGoalPose = AllianceFlipUtil.apply(goalPose);
            Rotation2d originalRotation = flippedGoalPose.getRotation();
            Rotation2d rotationAdjustment =
                AllianceFlipUtil.apply(getBranchPose(objective))
                    .getTranslation()
                    .minus(robot.get().getTranslation())
                    .getAngle()
                    .minus(originalRotation);
            boolean isLeftBranch = objective.branchId() % 2 == 0;
            rotationAdjustment =
                Rotation2d.fromDegrees(
                    MathUtil.clamp(
                        rotationAdjustment.getDegrees(),
                        isLeftBranch ? 0.0 : -maxAimingAngle.get(),
                        isLeftBranch ? maxAimingAngle.get() : 0.0));
            return new Pose2d(
                flippedGoalPose.getTranslation(), originalRotation.plus(rotationAdjustment));
          }
        };

    Container<Boolean> needsToGetBack = new Container<>(false);
    Container<Boolean> hasEnded = new Container<>(false);

    var driveCommand =
        new DriveToPose(
            drive,
            () ->
                coralObjective
                    .get()
                    .filter(
                        objective ->
                            !(superstructure.hasAlgae() && objective.reefLevel() == ReefLevel.L1))
                    .map(objective -> getDriveTarget(robot.get(), goal.apply(objective)))
                    .orElseGet(() -> drive.getPose()),
            () -> robot.get(),
            () ->
                DriveCommands.getLinearVelocityFromJoysticks(
                        driverX.getAsDouble(), driverY.getAsDouble())
                    .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
            () -> DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble()));

    Timer l4EjectTimer = new Timer();
    l4EjectTimer.start();
    return Commands.runOnce(
            () -> {
              // Start LEDs
              // Leds.getInstance().autoScoringReef = true;
              // Leds.getInstance().autoScoringLevel = reefLevel.get();

              // Reset state
              needsToGetBack.value = false;
              hasEnded.value = false;

              // Log reef level
              Logger.recordOutput("AutoScore/ReefLevel", reefLevel.get().toString());

              // Clear logs
              Logger.recordOutput("AutoScore/AllowReady", false);
              Logger.recordOutput("AutoScore/AllowEject", false);
            })
        .andThen(
            // Run superstructure
            preIntake(
                superstructure,
                () -> robot.get(),
                () -> reefLevel.get() == ReefLevel.L4,
                disableReefAutoAlign),
            // Check if need wait until pre ready or already ready
            Commands.waitUntil(
                () -> {
                  boolean ready =
                      coralObjective.get().isPresent()
                              && readyForSuperstructure(
                                  robot.get(),
                                  goal.apply(coralObjective.get().get()),
                                  reefLevel.get() == ReefLevel.L4)
                          || disableReefAutoAlign.getAsBoolean();
                  Logger.recordOutput("AutoScore/AllowReady", ready);

                  // Get back!
                  if (ready
                      && (reefLevel.get() == ReefLevel.L4 || superstructure.hasAlgae())
                      && !disableReefAutoAlign.getAsBoolean()
                      && DriverStation.isTeleopEnabled()) {
                    needsToGetBack.value = true;
                    superstructure.setReefDangerState(
                        Optional.of(Superstructure.getScoringState(reefLevel.get(), false)));
                  }
                  return ready;
                }),
            superstructureAimAndEject(
                    superstructure,
                    reefLevel,
                    coralObjective,
                    () -> {
                      if (coralObjective.get().isEmpty()) return false;
                      var reefPose = robot.get();

                      var objective = coralObjective.get().get();
                      Pose2d flippedRobot = AllianceFlipUtil.apply(reefPose);
                      Pose2d predictedRobot =
                          flippedRobot.exp(
                              drive
                                  .getChassisSpeeds()
                                  .toTwist2d(lookaheadEject[reefLevel.get().ordinal()].get()));

                      boolean ready =
                          checkEjectTolerances(
                                      flippedRobot,
                                      objective,
                                      reefLevel.get(),
                                      superstructure.hasAlgae())
                                  && checkEjectTolerances(
                                      predictedRobot,
                                      objective,
                                      reefLevel.get(),
                                      superstructure.hasAlgae())
                                  && superstructure.atGoal()
                                  && !disableReefAutoAlign.getAsBoolean()
                              || manualEject.getAsBoolean();
                      if (reefLevel.get() == ReefLevel.L4) {
                        if (!ready) {
                          l4EjectTimer.restart();
                        }
                        ready =
                            ready
                                && l4EjectTimer.hasElapsed(
                                    DriverStation.isAutonomous()
                                        ? l4EjectDelayAuto.get()
                                        : l4EjectDelay.get());
                      }
                      Logger.recordOutput("AutoScore/AllowEject", ready);
                      return ready;
                    },
                    manualEject,
                    disableReefAutoAlign)
                .andThen(
                    new ScheduleCommand(controllerRumble.withTimeout(controllerRumbleSecs)),
                    superstructure
                        .runGoal(() -> Superstructure.getScoringState(reefLevel.get(), false))
                        .until(() -> !disableReefAutoAlign.getAsBoolean())))
        .deadlineFor(
            Commands.either(
                joystickDrive, driveCommand, disableReefAutoAlign)) // Deadline with driving command
        .finallyDo(
            interrupted -> {
              // Clear logs
              Logger.recordOutput("AutoScore/ReefLevel", "");
              Logger.recordOutput("AutoScore/AllowPreReady", false);
              Logger.recordOutput("AutoScore/AllowEject", false);

              // Stop LEDs
              // Leds.getInstance().autoScoringReef = false;

              // Indicate has ended command
              hasEnded.value = true;
            });
  }

  public static Command autoScore(
      Drive drive,
      Superstructure superstructure,
      RollerSystem funnel,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective) {
    return autoScore(
        drive,
        superstructure,
        funnel,
        reefLevel,
        coralObjective,
        () -> 0,
        () -> 0,
        () -> 0,
        Commands.none(),
        Commands.none(),
        () -> false,
        () -> false,
        () -> false);
  }

  public static Command reefIntake(
      Drive drive,
      Superstructure superstructure,
      Optional<RollerSystem> funnel,
      Supplier<Optional<AlgaeObjective>> algaeObjective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Command joystickDrive,
      BooleanSupplier robotRelative,
      BooleanSupplier disableReefAutoAlign) {
    Supplier<Pose2d> robot = drive::getPose;

    Supplier<SuperstructureState> algaeIntakeState =
        () ->
            algaeObjective
                .get()
                .map(
                    objective ->
                        objective.id() % 2 == 0
                            ? SuperstructureState.ALGAE_L3_INTAKE
                            : SuperstructureState.ALGAE_L2_INTAKE)
                .orElseGet(superstructure::getState);

    Container<AlgaeObjective> algaeIntaked = new Container<>();
    Container<Boolean> needsToGetBack = new Container<>(false);
    Container<Boolean> hasEnded = new Container<>(false);
    Container<Boolean> complete = new Container<>(false);

    Timer hasAlgaeTimer = new Timer();
    hasAlgaeTimer.start();
    Supplier<Pose2d> goal =
        () ->
            algaeObjective
                .get()
                .map(AutoScoreCommands::getReefIntakePose)
                .orElseGet(
                    () ->
                        algaeIntaked.value == null
                            ? AllianceFlipUtil.apply(robot.get())
                            : getReefIntakePose(algaeIntaked.value));

    return Commands.runOnce(
            () -> {
              // Reset State
              algaeIntaked.value = null;
              needsToGetBack.value = false;
              hasEnded.value = false;
              complete.value = false;
            })
        .andThen(
            Commands.either(
                    joystickDrive,
                    new DriveToPose(
                        drive,
                        () -> {
                          Pose2d goalPose = goal.get();
                          if (algaeObjective.get().isEmpty() && !superstructure.hasAlgae()) {
                            return AllianceFlipUtil.apply(goalPose);
                          }
                          if (superstructure.getState() != algaeIntakeState.get()
                              && algaeObjective.get().isPresent()) {
                            goalPose =
                                goalPose.transformBy(
                                    GeomUtil.toTransform2d(
                                        -minDistanceReefClearAlgaeL4.get(), 0.0));
                          }
                          if (superstructure.hasAlgae()) {
                            if (hasAlgaeTimer.hasElapsed(algaeBackupTime.get())
                                && !disableReefAutoAlign.getAsBoolean()) {
                              complete.value = true;
                            }
                            goalPose =
                                goalPose.transformBy(
                                    GeomUtil.toTransform2d(
                                        -minDistanceReefClearAlgaeL4.get()
                                            * Math.min(
                                                1.0, hasAlgaeTimer.get() / algaeBackupTime.get()),
                                        0.0));
                          } else {
                            hasAlgaeTimer.restart();
                          }
                          return getDriveTarget(robot.get(), AllianceFlipUtil.apply(goalPose));
                        },
                        robot,
                        () ->
                            DriveCommands.getLinearVelocityFromJoysticks(
                                    driverX.getAsDouble(), driverY.getAsDouble())
                                .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
                        () -> DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble())),
                    disableReefAutoAlign)
                .alongWith(
                    (funnel.isEmpty()
                            ? Commands.none()
                            : preIntake(superstructure, robot, () -> false, disableReefAutoAlign))
                        .andThen(
                            // Check if need wait until pre ready or already ready
                            Commands.waitUntil(
                                () -> {
                                  boolean ready =
                                      readyForSuperstructure(
                                                  robot.get(),
                                                  AllianceFlipUtil.apply(goal.get()),
                                                  false)
                                              && algaeObjective.get().isPresent()
                                          || disableReefAutoAlign.getAsBoolean();
                                  Logger.recordOutput("ReefIntake/AllowReady", ready);
                                  // Get back!
                                  if (ready && DriverStation.isTeleopEnabled()) {
                                    needsToGetBack.value = true;
                                  }
                                  return ready;
                                }),
                            superstructure
                                .runGoal(algaeIntakeState)
                                .alongWith(
                                    Commands.waitUntil(
                                            () ->
                                                superstructure.getState() == algaeIntakeState.get())
                                        .andThen(
                                            () ->
                                                superstructure.setReefDangerState(
                                                    disableReefAutoAlign.getAsBoolean()
                                                        ? Optional.empty()
                                                        : Optional.of(algaeIntakeState.get()))))),
                    Commands.waitUntil(
                            () -> superstructure.hasAlgae() && algaeObjective.get().isPresent())
                        .andThen(
                            Commands.runOnce(
                                () -> {
                                  algaeIntaked.value = algaeObjective.get().get();
                                }))))
        .until(() -> complete.value)
        .finallyDo(
            () -> {
              complete.value = false;
              hasEnded.value = true;
            })
        .deadlineFor(
            Commands.run(() -> Logger.recordOutput("ReefIntake/Complete", complete.value)));
  }

  public static Command reefIntake(
      Drive drive,
      Superstructure superstructure,
      Supplier<Optional<AlgaeObjective>> algaeObjective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Command joystickDrive,
      BooleanSupplier robotRelative,
      BooleanSupplier disableReefAutoAlign) {
    return reefIntake(
        drive,
        superstructure,
        Optional.empty(),
        algaeObjective,
        driverX,
        driverY,
        driverOmega,
        joystickDrive,
        robotRelative,
        disableReefAutoAlign);
  }

  public static Command superAutoScore(
      Drive drive,
      Superstructure superstructure,
      RollerSystem funnel,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Supplier<Command> joystickDriveCommandFactory,
      Supplier<Command> controllerRumbleCommandFactory,
      BooleanSupplier robotRelative,
      BooleanSupplier disableReefAutoAlign,
      BooleanSupplier manualEject) {

    return reefIntake(
            drive,
            superstructure,
            Optional.of(funnel),
            () ->
                coralObjective.get().map(objective -> new AlgaeObjective(objective.branchId() / 2)),
            driverX,
            driverY,
            driverOmega,
            joystickDriveCommandFactory.get(),
            robotRelative,
            disableReefAutoAlign)
        .andThen(
            autoScore(
                drive,
                superstructure,
                funnel,
                reefLevel,
                coralObjective,
                driverX,
                driverY,
                driverOmega,
                joystickDriveCommandFactory.get(),
                controllerRumbleCommandFactory.get(),
                robotRelative,
                disableReefAutoAlign,
                manualEject))
        .beforeStarting(
            () -> {
              // Leds.getInstance().autoScoringReef = true;
              // Leds.getInstance().superAutoScoring = true;
              // Leds.getInstance().autoScoringLevel = reefLevel.get();
            })
        .finallyDo(
            () -> {
              // Leds.getInstance().autoScoringReef = false;
              // Leds.getInstance().superAutoScoring = false;
            });
  }

  public static Command superstructureAimAndEject(
      Superstructure superstructure,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective,
      BooleanSupplier eject,
      BooleanSupplier forceEject,
      BooleanSupplier disableReefAutoAlign) {
    final Timer ejectTimer = new Timer();
    return superstructure
        .runGoal(() -> Superstructure.getScoringState(reefLevel.get(), false))
        .until(eject)
        .andThen(
            Commands.runOnce(ejectTimer::restart),
            superstructure
                .runGoal(() -> Superstructure.getScoringState(reefLevel.get(), true))
                .until(
                    () ->
                        ejectTimer.hasElapsed(ejectTimeSeconds[reefLevel.get().levelNumber].get())
                            && !forceEject.getAsBoolean()));
  }

  public static Command superstructureAimAndEject(
      Superstructure superstructure,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective,
      BooleanSupplier eject) {
    return superstructureAimAndEject(
        superstructure, reefLevel, coralObjective, eject, () -> false, () -> false);
  }

  private static Command preIntake(
      Superstructure superstructure,
      Supplier<Pose2d> robot,
      BooleanSupplier shouldClearReef,
      BooleanSupplier disableReefAutoAlign) {
    return Commands.waitUntil(
            () ->
                outOfDistanceToReef(robot.get(), minDistanceReefClearL4.get())
                    || !shouldClearReef.getAsBoolean()
                    || disableReefAutoAlign.getAsBoolean())
        // .andThen(IntakeCommands.intake(superstructure, funnel).until(superstructure::hasCoral))
        .onlyIf(() -> !superstructure.hasCoral());
  }

  /** Get drive target. */
  public static Pose2d getDriveTarget(Pose2d robot, Pose2d goal) {
    Rotation2d angleToGoal =
        robot
            .getTranslation()
            .minus(AllianceFlipUtil.apply(Reef.center))
            .getAngle()
            .minus(goal.getTranslation().minus(AllianceFlipUtil.apply(Reef.center)).getAngle());
    Logger.recordOutput("AutoScore/AngleToGoal", angleToGoal);
    var offset = robot.relativeTo(goal);
    double yDistance = Math.abs(offset.getY());
    double xDistance = Math.abs(offset.getX());
    double shiftXT =
        MathUtil.clamp(
            (yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 4)),
            0.0,
            1.0);
    double shiftYT =
        MathUtil.clamp(yDistance <= 0.2 ? 0.0 : offset.getX() / Reef.faceLength, 0.0, 1.0);
    return goal.transformBy(
        GeomUtil.toTransform2d(
            -shiftXT * maxDistanceReefLineupX.get(),
            Math.copySign(shiftYT * maxDistanceReefLineupY.get(), offset.getY())));
  }

  /** Get position of robot aligned with branch for selected objective. */
  public static Pose2d getCoralScorePose(CoralObjective coralObjective) {
    int branchId = coralObjective.branchId();
    return getBranchPose(new CoralObjective(branchId, ReefLevel.L3))
        .transformBy(
            GeomUtil.toTransform2d(
                (switch (coralObjective.reefLevel()) {
                      case L2 -> l2ScoreDistance.get();
                      case L3 -> l3ScoreDistance.get();
                      case L4 -> l4ScoreDistance.get();
                      case L1 -> 0.0;
                    })
                    + Drive.DRIVE_BASE_LENGTH / 2.0,
                0.0))
        .transformBy(GeomUtil.toTransform2d(Rotation2d.kPi));
  }

  public static Pose2d getReefIntakePose(AlgaeObjective objective) {
    int branchId = objective.id() * 2;
    return getBranchPose(new CoralObjective(branchId, ReefLevel.L3))
        .interpolate(getBranchPose(new CoralObjective(branchId + 1, ReefLevel.L3)), 0.5)
        .transformBy(
            GeomUtil.toTransform2d(
                (objective.low() ? l2ReefIntakeDistance.get() : l3ReefIntakeDistance.get())
                    + Drive.DRIVE_BASE_LENGTH / 2.0,
                0.0))
        .transformBy(GeomUtil.toTransform2d(Rotation2d.kPi));
  }

  private static Pose2d getL1Pose(CoralObjective coralObjective) {
    Pose2d centerFace =
        Reef.centerFaces[coralObjective.branchId() / 2].transformBy(
            new Transform2d(
                l1AlignOffsetX.get(),
                l1AlignOffsetY.get(),
                Rotation2d.fromDegrees(l1AlignOffsetTheta.get())));
    return centerFace;
  }

  private static boolean checkEjectTolerances(
      Pose2d flippedRobot, CoralObjective objective, ReefLevel level, boolean algae) {
    // Separate tolerances for L1
    if (level == ReefLevel.L1) {
      var error = flippedRobot.relativeTo(getL1Pose(objective));
      return Math.abs(error.getX()) <= lowerXToleranceEject[level.ordinal()].get()
          && Math.abs(error.getY()) <= yToleranceEject[level.ordinal()].get()
          && Math.abs(error.getRotation().getDegrees()) <= l1ThetaEject.get();
    }

    var error = getCoralScorePose(objective).relativeTo(flippedRobot).getTranslation();
    double rawDistance = error.getNorm();
    double yError = Math.abs(error.getAngle().getTan() * rawDistance);
    double xError = Math.abs(rawDistance / error.getAngle().getCos());
    Logger.recordOutput("AutoScore/YError", yError);
    Logger.recordOutput("AutoScore/XError", xError);
    return yError <= yToleranceEject[level.ordinal()].get()
        && xError >= -lowerXToleranceEject[level.ordinal()].get()
        && xError <= upperXToleranceEject[level.ordinal()].get();
  }

  public static boolean readyForSuperstructure(Pose2d robot, Pose2d goal, boolean shouldBackUp) {
    double arcDistance =
        robot
                .getTranslation()
                .minus(AllianceFlipUtil.apply(Reef.center))
                .getAngle()
                .minus(goal.getTranslation().minus(AllianceFlipUtil.apply(Reef.center)).getAngle())
                .getRadians()
            * robot.getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center));
    return withinDistanceToReef(
            robot,
            DriverStation.isAutonomousEnabled()
                ? distanceSuperstructureReadyAuto.get()
                : distanceSuperstructureReady.get())
        && (outOfDistanceToReef(robot, minDistanceReefClearL4.get() - 0.1) || !shouldBackUp)
        && Math.abs(robot.relativeTo(goal).getRotation().getDegrees()) <= thetaToleranceReady.get()
        && Math.abs(arcDistance)
            <= (DriverStation.isAutonomousEnabled()
                ? arcDistanceReadyAuto.get()
                : arcDistanceReady.get());
  }

  public static boolean withinDistanceToReef(Pose2d robot, double distance) {
    final double distanceToReefCenter =
        AllianceFlipUtil.apply(robot).getTranslation().getDistance(Reef.center);
    Logger.recordOutput("AutoScore/DistanceToReefCenter", distanceToReefCenter);
    return distanceToReefCenter <= reefRadius + Drive.DRIVE_BASE_WIDTH / 2.0 + distance;
  }

  public static boolean outOfDistanceToReef(Pose2d robot, double distance) {
    final double distanceToReefCenter =
        AllianceFlipUtil.apply(robot).getTranslation().getDistance(Reef.center);
    Logger.recordOutput("AutoScore/DistanceToReefCenter", distanceToReefCenter);
    return distanceToReefCenter >= reefRadius + Drive.DRIVE_BASE_LENGTH / 2.0 + distance;
  }

  public static Pose2d getBranchPose(CoralObjective objective) {
    return Reef.branchPositions2d
        .get(objective.branchId())
        .get(objective.reefLevel())
        .transformBy(
            new Transform2d(
                branchFudgeX[objective.reefLevel().levelNumber].get(), 0, Rotation2d.kZero));
  }
}
