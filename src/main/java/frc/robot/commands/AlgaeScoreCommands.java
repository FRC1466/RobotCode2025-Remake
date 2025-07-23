// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive2.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Container;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.experimental.Accessors;

/**
 * AlgaeScoreCommands provides autonomous scoring functionality for the robot.
 *
 * <p>This class contains commands for processing game pieces and throwing game elements into
 * targets like the processor or net. It uses tunable parameters to adjust alignment and positioning
 * during these operations.
 *
 * <p>Key functionalities:
 *
 * <ul>
 *   <li>Process - Aligns robot with the processor for game piece scoring or ejection
 *   <li>Net Throw Lineup - Positions robot for throwing game elements
 *   <li>Net Throw Score - Executes the throwing sequence with coordinated drive and superstructure
 *       movements
 * </ul>
 *
 * <p>The class uses LoggedTunableNumbers for runtime adjustment of alignment offsets, distances,
 * tolerances, and velocities to fine-tune autonomous scoring behavior.
 *
 * @see Drive
 * @see Superstructure
 * @see SuperstructureState
 * @see DriveToPose
 * @see FieldConstants
 */
public class AlgaeScoreCommands {
  private static final LoggedTunableNumber processLineupXOffset =
      new LoggedTunableNumber("AlgaeScoreCommands/ProcessLineupXOffset", 0.1);
  private static final LoggedTunableNumber processLineupYOffset =
      new LoggedTunableNumber("AlgaeScoreCommands/ProcessLineupYOffset", 0.0);
  private static final LoggedTunableNumber processLineupClear =
      new LoggedTunableNumber("AlgaeScoreCommands/ProcessLineupClear", 0.3);
  private static final LoggedTunableNumber processEjectDegOffset =
      new LoggedTunableNumber("AlgaeScoreCommands/ProcessEjectDegreeOffset", 1.0);
  private static final LoggedTunableNumber throwLineupDistance =
      new LoggedTunableNumber("AlgaeScoreCommands/ThrowLineupDistance", 1.5);
  private static final LoggedTunableNumber throwDriveDistance =
      new LoggedTunableNumber("AlgaeScoreCommands/ThrowDriveDistance", 0.6);
  private static final LoggedTunableNumber throwDriveVelocity =
      new LoggedTunableNumber("AlgaeScoreCommands/ThrowDriveVelocity", 1.5);
  private static final LoggedTunableNumber throwReadyLinearTolerance =
      new LoggedTunableNumber("AlgaeScoreCommands/ThrowReadyLinearTolerance", 4.0);
  private static final LoggedTunableNumber throwReadyThetaToleranceDeg =
      new LoggedTunableNumber("AlgaeScoreCommands/ThrowReadyThetaToleranceDegrees", 40.0);
  private static final LoggedTunableNumber forceProcessorMaxDistance =
      new LoggedTunableNumber("AlgaeScoreCommands/ForceProcessorMaxDistance", 0.5);

  @Accessors(fluent = true)
  @Getter
  private static boolean shouldForceProcess = false;

  @Getter
  private static final LoggedTunableNumber lookaheadSecs =
      new LoggedTunableNumber("AlgaeScoreCommands/LookaheadSecs", 0.75);

  public static Command process(
      Drive drive,
      Superstructure superstructure,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Supplier<Command> joystickDrive,
      BooleanSupplier onOpposingSide,
      BooleanSupplier holdingButton,
      boolean eject,
      BooleanSupplier disableAlgaeScoreAutoAlign) {
    Container<Pose2d> goalPose = new Container<>(Pose2d.kZero);
    return Commands.either(
            joystickDrive.get(),
            Commands.run(
                    () -> {
                      goalPose.value =
                          AllianceFlipUtil.apply(
                                  onOpposingSide.getAsBoolean()
                                      ? FieldConstants.Processor.opposingCenterFace
                                      : FieldConstants.Processor.centerFace)
                              .transformBy(
                                  GeomUtil.toTransform2d(
                                      Drive.robotWidth / 2.0
                                          + processLineupXOffset.get()
                                          + (!eject
                                                  && superstructure.getState()
                                                      != SuperstructureState.PRE_PROCESS
                                              ? processLineupClear.get()
                                              : 0.0),
                                      processLineupYOffset.get()))
                              .transformBy(
                                  GeomUtil.toTransform2d(
                                      Rotation2d.kPi.plus(
                                          Rotation2d.fromDegrees(
                                              eject ? processEjectDegOffset.get() : 0.0))));
                      drive.setDesiredPoseForDriveToPoint(
                          AutoScoreCommands.getDriveTarget(
                              RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry(),
                              goalPose.value));
                    },
                    drive)
                .onlyWhile(holdingButton)
                .andThen(joystickDrive.get()),
            disableAlgaeScoreAutoAlign)
        .alongWith(
            eject
                ? superstructure.runGoal(SuperstructureState.PROCESS)
                : superstructure.runGoal(SuperstructureState.PRE_PROCESS),
            Commands.run(
                () ->
                    shouldForceProcess =
                        !disableAlgaeScoreAutoAlign.getAsBoolean()
                            && superstructure.getState() == SuperstructureState.PRE_PROCESS
                            && RobotState.getInstance()
                                    .getRobotPoseFromSwerveDriveOdometry()
                                    .getTranslation()
                                    .getDistance(goalPose.value.getTranslation())
                                < forceProcessorMaxDistance.get()))
        .finallyDo(() -> shouldForceProcess = false);
  }

  public static Command netThrowLineup(
      Drive drive,
      Superstructure superstructure,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      Command joystickDrive,
      BooleanSupplier disableAlgaeScoreAutoAlign) {
    var autoAlignCommand =
        Commands.run(
            () -> {
              Pose2d targetPose =
                  new Pose2d(
                      AllianceFlipUtil.applyX(
                          FieldConstants.fieldLength / 2.0 - throwLineupDistance.get()),
                      RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry().getY(),
                      AllianceFlipUtil.apply(Rotation2d.k180deg));
              drive.setDesiredPoseForDriveToPoint(
                  AutoScoreCommands.getDriveTarget(
                      RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry(), targetPose));
            },
            drive);

    return Commands.either(joystickDrive, autoAlignCommand, disableAlgaeScoreAutoAlign)
        .alongWith(
            Commands.waitUntil(
                    () ->
                        disableAlgaeScoreAutoAlign.getAsBoolean()
                            || (drive.getSystemState() == Drive.SystemState.DRIVE_TO_POINT
                                && drive.isAtDriveToPointSetpoint(
                                    throwReadyLinearTolerance.get(),
                                    Rotation2d.fromDegrees(throwReadyThetaToleranceDeg.get()))))
                .andThen(superstructure.runGoal(SuperstructureState.PRE_THROW)));
  }

  public static Command netThrowScore(Drive drive, Superstructure superstructure) {
    Container<Pose2d> startPose = new Container<>();
    Timer driveTimer = new Timer();
    return Commands.runOnce(
            () -> {
              startPose.value = RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry();
              driveTimer.restart();
            })
        .andThen(
            Commands.run(
                    () -> {
                      Pose2d targetPose =
                          startPose.value.transformBy(
                              GeomUtil.toTransform2d(
                                  -Math.min(
                                      driveTimer.get() * throwDriveVelocity.get(),
                                      throwDriveDistance.get()),
                                  0));
                      drive.setDesiredPoseForDriveToPoint(targetPose);
                    },
                    drive)
                .alongWith(
                    superstructure.runGoal(
                        () ->
                            driveTimer.get() * throwDriveVelocity.get() > throwDriveDistance.get()
                                ? SuperstructureState.THROW
                                : SuperstructureState.PRE_THROW)));
  }
}
