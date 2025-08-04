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
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.AlgaeObjective;
import frc.robot.constants.FieldConstants.CoralObjective;
import frc.robot.constants.FieldConstants.IceCreamObjective;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.*;
import java.util.Optional;
import java.util.function.*;
import org.littletonrobotics.junction.Logger;

/**
 * Commands for automatically scoring game pieces on the field reef structures.
 *
 * <p>This class provides a collection of autonomous and assisted commands for: - Auto-scoring coral
 * at different reef levels (L1-L4) - Auto-intake of algae from reef branches - Auto-intake of algae
 * from ice cream positions - Super auto-scoring that combines coral scoring and reef intake
 *
 * <p>The commands handle alignment, approach, and automated superstructure sequencing using
 * configurable tolerances and parameters through LoggedTunableNumbers.
 *
 * <p>Key features: - Path planning and targeting relative to field elements - Automatic pose
 * adjustment based on robot position and reef geometry - Superstructure state management during
 * scoring sequences - Eject timing and positioning tolerances for reliable scoring - Manual
 * override capabilities through driver inputs - Alliance-aware positioning (automatic flipping
 * based on alliance color)
 *
 * <p>All alignment tolerances, distances, and timing parameters are tunable through NetworkTables
 * using the LoggedTunableNumber system.
 */
public class AutoScoreCommands {
  // Radius of regular hexagon is side length
  private static final double reefRadius = Reef.faceLength;
  private static final double controllerRumbleSecs = 0.2;

  private static final LoggedTunableNumber maxDistanceReefLineupX =
      new LoggedTunableNumber("AutoScore/MaxDistanceReefLineupX", 1.0);
  private static final LoggedTunableNumber maxDistanceReefLineupY =
      new LoggedTunableNumber("AutoScore/MaxDistanceReefLineupY", 1.2);
  public static final LoggedTunableNumber minDistanceReefClearL4 =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClear", Units.inchesToMeters(6.0));
  public static final LoggedTunableNumber minDistanceReefClearAlgaeL4 =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClearAlgae", 1);
  public static final LoggedTunableNumber minAngleReefClear =
      new LoggedTunableNumber("AutoScore/MinAngleReefClear", 30.0);
  public static final LoggedTunableNumber algaeBackupTime =
      new LoggedTunableNumber("AutoScore/AlgaeBackupTime", 0.75);
  private static final LoggedTunableNumber distanceSuperstructureReady =
      new LoggedTunableNumber("AutoScore/DistanceSuperstructureReady", 2.0);
  private static final LoggedTunableNumber distanceSuperstructureReadyAuto =
      new LoggedTunableNumber("AutoScore/DistanceSuperstructureReadyAuto", 2.0);
  private static final LoggedTunableNumber thetaToleranceReady =
      new LoggedTunableNumber("AutoScore/ThetaToleranceReady", 35.0);
  private static final LoggedTunableNumber arcDistanceReady =
      new LoggedTunableNumber("AutoScore/ArcDistanceReady", 1.5);
  private static final LoggedTunableNumber arcDistanceReadyAuto =
      new LoggedTunableNumber("AutoScore/ArcDistanceReadyAuto", 1.5);
  private static final LoggedTunableNumber[] lowerXToleranceEject = {
    new LoggedTunableNumber("AutoScore/XToleranceEject/Lower/L1", 0.03),
    new LoggedTunableNumber("AutoScore/XToleranceEject/Lower/L2", 0.05),
    new LoggedTunableNumber("AutoScore/XToleranceEject/Lower/L3", 0.05),
    new LoggedTunableNumber("AutoScore/XToleranceEject/Lower/L4", 0.04)
  };
  private static final LoggedTunableNumber[] upperXToleranceEject = {
    new LoggedTunableNumber("AutoScore/XToleranceEject/Upper/L1", 0.03),
    new LoggedTunableNumber("AutoScore/XToleranceEject/Upper/L2", 0.08),
    new LoggedTunableNumber("AutoScore/XToleranceEject/Upper/L3", 0.08),
    new LoggedTunableNumber("AutoScore/XToleranceEject/Upper/L4", 0.04)
  };
  private static final LoggedTunableNumber[] yToleranceEject = {
    new LoggedTunableNumber("AutoScore/YToleranceEject/L1", 0.04),
    new LoggedTunableNumber("AutoScore/YToleranceEject/L2", 0.04),
    new LoggedTunableNumber("AutoScore/YToleranceEject/L3", 0.04),
    new LoggedTunableNumber("AutoScore/YToleranceEject/L4", 0.03)
  };
  private static final LoggedTunableNumber[] lookaheadEject = {
    new LoggedTunableNumber("AutoScore/LookaheadToleranceEject/L1", 0.5),
    new LoggedTunableNumber("AutoScore/LookaheadToleranceEject/L2", 0.3),
    new LoggedTunableNumber("AutoScore/LookaheadToleranceEject/L3", 0.3),
    new LoggedTunableNumber("AutoScore/LookaheadToleranceEject/L4", 0.3)
  };
  private static final LoggedTunableNumber l1ThetaEject =
      new LoggedTunableNumber("AutoScore/L1ThetaToleranceEject", 5.0);
  private static final LoggedTunableNumber l4EjectDelay =
      new LoggedTunableNumber("AutoScore/L4EjectDelay", 0.5);
  private static final LoggedTunableNumber l4EjectDelayAuto =
      new LoggedTunableNumber("AutoScore/L4EjectDelayAuto", 0.2);
  private static final LoggedTunableNumber l2ReefIntakeDistance =
      new LoggedTunableNumber("AutoScore/L2ReefIntakeDistance", 0);
  private static final LoggedTunableNumber l3ReefIntakeDistance =
      new LoggedTunableNumber("AutoScore/L3ReefIntakeDistance", 0);
  private static final LoggedTunableNumber maxAimingAngle =
      new LoggedTunableNumber("AutoScore/MaxAimingAngle", 20.0);
  private static final LoggedTunableNumber l1AlignOffsetX =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetX", 0.625);
  private static final LoggedTunableNumber l1AlignOffsetY =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetY", 0.0);
  private static final LoggedTunableNumber l1AlignOffsetTheta =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetTheta", 180.0);
  private static final LoggedTunableNumber[] ejectTimeSeconds = {
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L1", 3),
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L2", 0.3),
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L3", 0.3),
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L4", 0.3)
  };
  private static final LoggedTunableNumber[] branchFudgeX = {
    new LoggedTunableNumber("AutoScore/FudgeX/L1", 0.0),
    new LoggedTunableNumber("AutoScore/FudgeX/L2", 0.0),
    new LoggedTunableNumber("AutoScore/FudgeX/L3", 0.0),
    new LoggedTunableNumber("AutoScore/FudgeX/L4", 0.02)
  };

  private static final LoggedTunableNumber l2ScoreDistance =
      new LoggedTunableNumber("AutoScore/L2ScoreDistance", 0.1);
  private static final LoggedTunableNumber l3ScoreDistance =
      new LoggedTunableNumber("AutoScore/L3ScoreDistance", 0.1);
  private static final LoggedTunableNumber l4ScoreDistance =
      new LoggedTunableNumber("AutoScore/L4ScoreDistance", 0.15);

  private AutoScoreCommands() {}

  public static Command autoScore(
      Drive drive,
      Superstructure superstructure,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Command joystickDrive,
      Command controllerRumble,
      BooleanSupplier disableReefAutoAlign,
      BooleanSupplier manualEject) {
    Supplier<Pose2d> drivePoseSupplier =
        () -> RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry();

    Function<CoralObjective, Pose2d> goal =
        objective -> {
          boolean hasAlgae = superstructure.hasAlgae();
          if (reefLevel.get() == ReefLevel.L1) {
            return AllianceFlipUtil.apply(getL1Pose(objective));
          }
          Pose2d goalPose =
              getCoralScorePose(new CoralObjective(objective.branchId(), reefLevel.get()));
          Pose2d currentRobotPose = drivePoseSupplier.get();
          final double clearDistance =
              hasAlgae ? minDistanceReefClearAlgaeL4.get() : minDistanceReefClearL4.get();
          double translateX = 0.0;
          if ((!superstructure.readyForL4() && reefLevel.get() == ReefLevel.L4)
              || (superstructure.readyForL4()
                  && withinDistanceToReef(currentRobotPose, clearDistance - 0.05)
                  && reefLevel.get() != ReefLevel.L4)) {
            translateX =
                Math.min(
                    translateX,
                    goalPose.getTranslation().getDistance(Reef.center)
                        - reefRadius
                        - (Drive.robotWidth / 2.0)
                        - clearDistance);
          }
          goalPose = goalPose.transformBy(GeomUtil.toTransform2d(translateX, 0.0));
          Logger.recordOutput("AutoScore/Goal", AllianceFlipUtil.apply(goalPose));
          Pose2d flippedGoalPose = AllianceFlipUtil.apply(goalPose);
          Rotation2d originalRotation = flippedGoalPose.getRotation();
          Rotation2d rotationAdjustment =
              AllianceFlipUtil.apply(getBranchPose(objective))
                  .getTranslation()
                  .minus(currentRobotPose.getTranslation())
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
        };

    Container<Boolean> needsToGetBack = new Container<>(false);
    Container<Boolean> hasEnded = new Container<>(false);

    var driveCommand =
        Commands.run(
            () -> {
              var targetPose =
                  coralObjective
                      .get()
                      .filter(
                          objective ->
                              !(superstructure.hasAlgae() && objective.reefLevel() == ReefLevel.L1))
                      .map(
                          objective ->
                              getDriveTarget(drivePoseSupplier.get(), goal.apply(objective)))
                      .orElseGet(
                          () -> RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry());
              drive.setDesiredPoseForDriveToPoint(targetPose);
            });

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
              Logger.recordOutput("AutoScore/AllowEject", false);
            })
        .andThen(
            // Run superstructure
            Commands.waitUntil(superstructure::hasCoral),
            // Check if need wait until pre ready or already ready
            Commands.waitUntil(
                () -> {
                  boolean ready =
                      coralObjective.get().isPresent()
                              && readyForSuperstructure(
                                  drivePoseSupplier.get(),
                                  goal.apply(coralObjective.get().get()),
                                  reefLevel.get() == ReefLevel.L4)
                          || disableReefAutoAlign.getAsBoolean();
                  Logger.recordOutput("AutoScore/AllowReady", ready);

                  // Get back!
                  if (ready
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
                      Pose2d currentRobotPose = drivePoseSupplier.get();

                      var objective = coralObjective.get().get();
                      Pose2d flippedRobot = AllianceFlipUtil.apply(currentRobotPose);
                      Pose2d predictedRobot =
                          flippedRobot.exp(
                              RobotState.getInstance()
                                  .getRobotChassisSpeeds()
                                  .toTwist2d(lookaheadEject[reefLevel.get().ordinal()].get()));

                      boolean ready =
                          (checkEjectTolerances(
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
                              || manualEject.getAsBoolean());
                      if (!ready) {
                        l4EjectTimer.restart();
                      }
                      ready =
                          ready
                              && l4EjectTimer.hasElapsed(
                                  DriverStation.isAutonomous()
                                      ? l4EjectDelayAuto.get()
                                      : l4EjectDelay.get());

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
                joystickDrive,
                driveCommand,
                disableReefAutoAlign)) // Just drive if auto align disabled
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
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective) {
    return autoScore(
        drive,
        superstructure,
        reefLevel,
        coralObjective,
        () -> 0,
        () -> 0,
        () -> 0,
        Commands.none(),
        Commands.none(),
        () -> false,
        () -> false);
  }

  public static Command reefIntake(
      Drive drive,
      Superstructure superstructure,
      Supplier<Optional<AlgaeObjective>> algaeObjective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Command joystickDrive,
      BooleanSupplier disableReefAutoAlign,
      Boolean isSuper) {
    Supplier<Pose2d> drivePoseSupplier =
        () -> RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry();

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
                            ? AllianceFlipUtil.apply(drivePoseSupplier.get())
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
                    Commands.run(
                        () -> {
                          Pose2d goalPose = goal.get();
                          if (algaeObjective.get().isEmpty() && !superstructure.hasAlgae()) {
                            drive.setDesiredPoseForDriveToPoint(AllianceFlipUtil.apply(goalPose));
                            return;
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
                          drive.setDesiredPoseForDriveToPoint(
                              getDriveTarget(
                                  drivePoseSupplier.get(), AllianceFlipUtil.apply(goalPose)));
                        }),
                    disableReefAutoAlign)
                .alongWith(
                    // Check if need wait until pre ready or already ready
                    Commands.waitUntil(
                        () -> {
                          boolean ready =
                              readyForSuperstructure(
                                          drivePoseSupplier.get(),
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
                    isSuper
                        ? Commands.waitUntil(
                                () ->
                                    outOfDistanceToReef(
                                        drivePoseSupplier.get(), Units.inchesToMeters(8)))
                            .andThen(superstructure.runGoal(algaeIntakeState))
                        : superstructure.runGoal(algaeIntakeState),
                    Commands.waitUntil(
                        () -> superstructure.hasAlgae() && algaeObjective.get().isPresent()))
                .andThen(
                    Commands.runOnce(
                        () -> {
                          algaeIntaked.value = algaeObjective.get().get();
                        })))
        .until(() -> complete.value)
        .finallyDo(
            () -> {
              complete.value = false;
              hasEnded.value = true;
            })
        .deadlineFor(
            Commands.run(() -> Logger.recordOutput("ReefIntake/Complete", complete.value)));
  }

  public static Command iceCreamIntake(
      Drive drive,
      Superstructure superstructure,
      Supplier<Optional<IceCreamObjective>> iceCreamObjective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Command joystickDrive,
      BooleanSupplier disableIceCreamAutoAlign,
      BooleanSupplier intake) {

    Supplier<Pose2d> robot = RobotState.getInstance()::getRobotPoseFromSwerveDriveOdometry;

    Supplier<SuperstructureState> iceCreamIntakeState =
        () -> SuperstructureState.ALGAE_ICE_CREAM_INTAKE;

    Container<Boolean> complete = new Container<>(false);
    Timer hasIceCreamTimer = new Timer();
    hasIceCreamTimer.start();

    Supplier<Pose2d> goal =
        () ->
            iceCreamObjective.get().map(AutoScoreCommands::getIceCreamIntakePose).orElseGet(robot);

    Timer intakeDriveTimer = new Timer();

    return Commands.runOnce(
            () -> {
              complete.value = false;
              intakeDriveTimer.reset();
              intakeDriveTimer.stop();
            })
        .andThen(
            Commands.either(
                    joystickDrive,
                    Commands.run(
                        () -> {
                          Pose2d goalPose = goal.get();
                          if (superstructure.hasAlgae()) {
                            if (hasIceCreamTimer.hasElapsed(algaeBackupTime.get())
                                && !disableIceCreamAutoAlign.getAsBoolean()) {
                              complete.value = true;
                            }
                          } else {
                            hasIceCreamTimer.restart();
                          }

                          // If intake is pressed, start timer and drive forward slowly
                          if (intake.getAsBoolean()) {
                            if (!intakeDriveTimer.isRunning()) {
                              intakeDriveTimer.restart();
                            }
                            // Move forward based on timer (similar to netThrowScore approach)
                            double forwardDistance = Math.min(intakeDriveTimer.get() * 1, .6);
                            goalPose =
                                goalPose.transformBy(GeomUtil.toTransform2d(forwardDistance, 0.0));
                          } else {
                            intakeDriveTimer.stop();
                            intakeDriveTimer.reset();
                          }

                          drive.setDesiredPoseForDriveToPoint(
                              getDriveTarget(robot.get(), AllianceFlipUtil.apply(goalPose)));
                        }),
                    disableIceCreamAutoAlign)
                .alongWith(
                    Commands.waitUntil(
                        () -> {
                          boolean ready =
                              readyForSuperstructure(
                                      robot.get(), AllianceFlipUtil.apply(goal.get()), false)
                                  || disableIceCreamAutoAlign.getAsBoolean();
                          Logger.recordOutput("IceCreamIntake/AllowReady", ready);
                          return ready;
                        }),
                    superstructure.runGoal(iceCreamIntakeState)),
            Commands.waitUntil(() -> superstructure.hasAlgae())
                .andThen(
                    Commands.runOnce(
                        () -> {
                          Logger.recordOutput(
                              "IceCreamIntake/Position",
                              iceCreamObjective
                                  .get()
                                  .map(
                                      obj -> {
                                        switch (obj.iceCreamPosition()) {
                                          case 1:
                                            return "right";
                                          case 2:
                                            return "middle";
                                          case 3:
                                            return "left";
                                          default:
                                            return "unknown";
                                        }
                                      })
                                  .orElse("none"));
                        })))
        .until(() -> complete.value)
        .finallyDo(
            () -> {
              complete.value = false;
              intakeDriveTimer.stop();
              intakeDriveTimer.reset();
            })
        .deadlineFor(
            Commands.run(() -> Logger.recordOutput("IceCreamIntake/Complete", complete.value)));
  }

  public static Command superAutoScore(
      Drive drive,
      Superstructure superstructure,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Supplier<Command> joystickDriveCommandFactory,
      Supplier<Command> controllerRumbleCommandFactory,
      BooleanSupplier disableReefAutoAlign,
      BooleanSupplier manualEject) {

    return autoScore(
            drive,
            superstructure,
            reefLevel,
            coralObjective,
            driverX,
            driverY,
            driverOmega,
            joystickDriveCommandFactory.get(),
            controllerRumbleCommandFactory.get(),
            disableReefAutoAlign,
            manualEject)
        .andThen(
            reefIntake(
                drive,
                superstructure,
                () ->
                    coralObjective
                        .get()
                        .map(objective -> new AlgaeObjective(objective.branchId() / 2)),
                driverX,
                driverY,
                driverOmega,
                joystickDriveCommandFactory.get(),
                disableReefAutoAlign,
                true))
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
    return getBranchPose(new CoralObjective(coralObjective.branchId(), ReefLevel.L3))
        .transformBy(
            GeomUtil.toTransform2d(
                (switch (coralObjective.reefLevel()) {
                      case L2 -> l2ScoreDistance.get();
                      case L3 -> l3ScoreDistance.get();
                      case L4 -> l4ScoreDistance.get();
                      default -> 0.1;
                    })
                    + Drive.robotWidth / 2.0,
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
                    + Drive.robotWidth / 2.0,
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
    if (level == ReefLevel.L1) {
      var error = flippedRobot.relativeTo(getL1Pose(objective));
      return Math.abs(error.getX()) <= lowerXToleranceEject[level.ordinal()].get()
          && Math.abs(error.getY()) <= yToleranceEject[level.ordinal()].get()
          && Math.abs(error.getRotation().getDegrees()) <= l1ThetaEject.get();
    }

    Pose2d targetScorePose = getCoralScorePose(new CoralObjective(objective.branchId(), level));
    var errorInGoalFrame = flippedRobot.relativeTo(targetScorePose);
    double xError = errorInGoalFrame.getX();
    double yError = Math.abs(errorInGoalFrame.getY());

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
    return distanceToReefCenter <= reefRadius + Drive.robotWidth / 2.0 + distance;
  }

  public static boolean outOfDistanceToReef(Pose2d robot, double distance) {
    final double distanceToReefCenter =
        AllianceFlipUtil.apply(robot).getTranslation().getDistance(Reef.center);
    Logger.recordOutput("AutoScore/DistanceToReefCenter", distanceToReefCenter);
    return distanceToReefCenter >= reefRadius + Drive.robotWidth / 2.0 + distance;
  }

  public static Pose2d getBranchPose(CoralObjective objective) {
    return Reef.branchPositions2d
        .get(objective.branchId())
        .get(objective.reefLevel())
        .transformBy(
            new Transform2d(
                branchFudgeX[objective.reefLevel().levelNumber].get(), 0, Rotation2d.kZero));
  }

  public static Pose2d getIceCreamIntakePose(IceCreamObjective objective) {
    return FieldConstants.iceCreamPositions
        .get(objective)
        .transformBy(new Transform2d(1, 0, Rotation2d.kPi));
  }
}
