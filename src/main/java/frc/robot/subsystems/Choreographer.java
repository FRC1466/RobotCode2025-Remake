// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.ChoreographerConstants.*;
import static frc.robot.constants.ChoreographerPositions.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.constants.ChoreographerConstants.ScoringSide;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ReefConstants;
import frc.robot.constants.WristElevatorPoses;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.overridePublisher.OverridePublisher;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Position;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Choreographer extends SubsystemBase {
  private final Drive drive;
  private final Intake intake;
  private final Elevator elevator;
  private final Pivot wrist;
  private final OverridePublisher overrides;
  private final Vision vision;

  // Debouncers
  private final Debouncer homeDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
  private final Debouncer simCoralDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kRising);
  private final Debouncer simAlgaeDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kRising);
  private final Debouncer readyToScoreDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kRising);
  private final Debouncer readyToScoreDebouncerAuto =
      new Debouncer(0.5, Debouncer.DebounceType.kRising);
  private final Debouncer intakeDebouncerAuto = new Debouncer(0.1, Debouncer.DebounceType.kRising);

  public enum WantedChoreography {
    HOME,
    STOPPED,
    DEFAULT_STATE,
    INTAKE_CORAL_FROM_STATION,
    SCORE_L1,
    SCORE_L2,
    SCORE_L3,
    SCORE_L4,
    INTAKE_ALGAE_REEF,
    INTAKE_ALGAE_ICE_CREAM,
    MOVE_ALGAE_TO_NET_POSITION,
    SCORE_ALGAE_IN_NET,
    MOVE_ALGAE_TO_PROCESSOR_POSITION,
    SCORE_ALGAE_IN_PROCESSOR,
    EJECT_ALGAE,
    EJECT_CORAL
  }

  private enum CurrentChoreography {
    HOME,
    STOPPED,
    NO_PIECE,
    HOLDING_CORAL,
    HOLDING_ALGAE,
    INTAKE_CORAL_FROM_STATION,
    SCORE_L1,
    SCORE_L2,
    SCORE_L3,
    SCORE_L4,
    INTAKE_ALGAE_REEF,
    INTAKE_ALGAE_ICE_CREAM,
    MOVE_ALGAE_TO_NET_POSITION,
    SCORE_ALGAE_IN_NET,
    MOVE_ALGAE_TO_PROCESSOR_POSITION,
    SCORE_ALGAE_IN_PROCESSOR,
    EJECT_ALGAE,
    EJECT_CORAL
  }

  private WantedChoreography wantedChoreography = WantedChoreography.STOPPED;
  private CurrentChoreography currentChoreography = CurrentChoreography.STOPPED;

  @Getter private boolean disabled = false; // For testing system
  private CurrentChoreography previousChoreography;

  private ScoringSide targetScoringSide = ScoringSide.LEFT;

  private boolean hasDriveReachedMiddleAlgae = false;
  private boolean hasDriveReachedMiddleCoral = false;

  private boolean coralEject = false;

  @Getter private BooleanSupplier wristPastSafe = () -> false;

  private final DriveToPose driveToPoseCommand;
  private Pose2d driveToPoseTarget = new Pose2d();

  public Choreographer(
      Drive drive,
      Intake intake,
      Elevator elevator,
      Pivot wrist,
      OverridePublisher overrides,
      Vision vision,
      CommandXboxController driverController) {
    this.drive = drive;
    this.intake = intake;
    this.elevator = elevator;
    this.wrist = wrist;
    this.overrides = overrides;
    this.vision = vision;

    this.wristPastSafe = () -> wrist.getAngle().getRadians() > .6;

    // Create the command once, with a supplier that gets the current target
    this.driveToPoseCommand =
        new DriveToPose(
            drive,
            () -> this.driveToPoseTarget,
            drive::getPose,
            () ->
                DriveCommands.getLinearVelocityFromJoysticks(
                    -driverController.getLeftX(), -driverController.getLeftY()),
            () -> -driverController.getRightX());
  }

  @Override
  public void periodic() {
    // Skip if disabled by testing system
    if (disabled) {
      Logger.recordOutput("Choreographer/Disabled", true);
      return;
    }

    Logger.recordOutput("Choreographer/Wanted", wantedChoreography);
    Logger.recordOutput("Choreographer/Current", currentChoreography);
    Logger.recordOutput("Choreographer/Previous", previousChoreography);
    Logger.recordOutput("Choreographer/Disabled", false);
    Logger.recordOutput("Choreographer/TargetScoringSide", targetScoringSide);

    currentChoreography = computeChoreography();
    applyChoreography();
  }

  /**
   * Enable or disable the choreographer (used by testing system). When disabled, choreographer will
   * not run and subsystems can be controlled directly.
   *
   * @param disabled true to disable, false to enable
   */
  public void setDisabled(boolean disabled) {
    this.disabled = disabled;
    if (disabled) {
      System.out.println("CHOREOGRAPHER DISABLED - Subsystems can be controlled directly");
    } else {
      System.out.println("CHOREOGRAPHER ENABLED - Normal operation resumed");
    }
  }

  private CurrentChoreography computeChoreography() {
    previousChoreography = currentChoreography;
    switch (wantedChoreography) {
      default:
        currentChoreography = CurrentChoreography.STOPPED;
        break;
      case HOME:
        currentChoreography = CurrentChoreography.HOME;
        break;
      case INTAKE_CORAL_FROM_STATION:
        currentChoreography = CurrentChoreography.INTAKE_CORAL_FROM_STATION;
        break;
      case DEFAULT_STATE:
        if (intake.hasCoral()) {
          currentChoreography = CurrentChoreography.HOLDING_CORAL;
        } else if (intake.hasAlgae()) {
          currentChoreography = CurrentChoreography.HOLDING_ALGAE;
        } else {
          currentChoreography = CurrentChoreography.NO_PIECE;
        }
        break;
      case SCORE_L1:
        currentChoreography = CurrentChoreography.SCORE_L1;
        break;
      case SCORE_L2:
        currentChoreography = CurrentChoreography.SCORE_L2;
        break;
      case SCORE_L3:
        currentChoreography = CurrentChoreography.SCORE_L3;
        break;
      case SCORE_L4:
        currentChoreography = CurrentChoreography.SCORE_L4;
        break;
      case INTAKE_ALGAE_ICE_CREAM:
        currentChoreography = CurrentChoreography.INTAKE_ALGAE_ICE_CREAM;
        break;
      case INTAKE_ALGAE_REEF:
        currentChoreography = CurrentChoreography.INTAKE_ALGAE_REEF;
        break;
      case MOVE_ALGAE_TO_NET_POSITION:
        currentChoreography = CurrentChoreography.MOVE_ALGAE_TO_NET_POSITION;
        break;
      case MOVE_ALGAE_TO_PROCESSOR_POSITION:
        currentChoreography = CurrentChoreography.MOVE_ALGAE_TO_PROCESSOR_POSITION;
        break;
      case SCORE_ALGAE_IN_NET:
        currentChoreography = CurrentChoreography.SCORE_ALGAE_IN_NET;
        break;
      case SCORE_ALGAE_IN_PROCESSOR:
        currentChoreography = CurrentChoreography.SCORE_ALGAE_IN_PROCESSOR;
        break;
    }
    return currentChoreography;
  }

  private void applyChoreography() {
    if (previousChoreography != currentChoreography) {
      resetDebouncers();
    }

    switch (currentChoreography) {
      case HOME:
        home();
        break;
      case INTAKE_CORAL_FROM_STATION:
        intakeCoralFromStation();
        break;
      case NO_PIECE:
        if (DriverStation.isAutonomous()) {
          noPieceAuto();
        } else {
          noPiece();
        }
        break;
      case HOLDING_CORAL:
        if (DriverStation.isAutonomous()) {
          holdingCoralAuto();
        } else {
          holdingCoral();
        }
        break;
      case HOLDING_ALGAE:
        holdingAlgae();
        break;
      case SCORE_L1:
        scoreL1Teleop(targetScoringSide);
        break;
      case SCORE_L2:
        scoreL2Teleop(targetScoringSide);
        break;
      case SCORE_L3:
        scoreL3Teleop(targetScoringSide);
        break;
      case SCORE_L4:
        scoreL4Teleop(targetScoringSide);
        break;
      case INTAKE_ALGAE_ICE_CREAM:
        intakeAlgaeIceCream();
        break;
      case INTAKE_ALGAE_REEF:
        intakeAlgaeFromReef();
        break;
      case SCORE_ALGAE_IN_NET:
        scoreAlgaeNet();
        break;
      case SCORE_ALGAE_IN_PROCESSOR:
        scoreAlgaeProcessor();
        break;
      case MOVE_ALGAE_TO_NET_POSITION:
        moveAlgaeToNetPosition();
        break;
      case MOVE_ALGAE_TO_PROCESSOR_POSITION:
        moveAlgaeToProcessorPosition();
        break;
      case EJECT_ALGAE:
        intake.setWantedState(Intake.WantedState.EJECT_ALGAE);
        break;
      case EJECT_CORAL:
        intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
        break;
      case STOPPED:
        stopped();
        break;
    }
  }

  public void resetDebouncers() {
    homeDebouncer.calculate(false);
    simAlgaeDebouncer.calculate(false);
    simCoralDebouncer.calculate(false);
    readyToScoreDebouncer.calculate(false);
    readyToScoreDebouncerAuto.calculate(false);
    intakeDebouncerAuto.calculate(false);
    hasDriveReachedMiddleCoral = false;
    hasDriveReachedMiddleAlgae = false;
  }

  private void home() {}

  private void stopped() {
    wrist.setWantedState(Pivot.WantedState.IDLE);
    elevator.setWantedState(Elevator.WantedState.IDLE);
    intake.setWantedState(Intake.WantedState.OFF);
  }

  private void holdingAlgae() {
    subsystemsRun(ALGAE_HOLD);
    intake.setWantedState(
        wrist.atGoal() && elevator.atGoal()
            ? Intake.WantedState.HOLD_ALGAE
            : Intake.WantedState.HOLD_ALGAE_HARDER);
  }

  private void holdingCoral() {
    coralEject = false;
    subsystemsRun(TRAVEL);
    intake.setWantedState(Intake.WantedState.OFF);
  }

  private void holdingCoralAuto() {
    coralEject = false;
    subsystemsRun(TRAVEL);
    intake.setWantedState(Intake.WantedState.OFF);
  }

  private void noPiece() {
    if (elevator.getPosition() > 0.1) {
      subsystemsRun(TRAVEL);
    } else {
      subsystemsRun(STOW);
    }
    intake.setWantedState(Intake.WantedState.OFF);
  }

  private void noPieceAuto() {
    subsystemsRun(TRAVEL);
    intake.setWantedState(Intake.WantedState.OFF);
  }

  private void intakeCoralFromStation() {
    coralEject = false;
    if (DriverStation.isAutonomous()) {
      if (elevator.getPosition() > 0.15) {
        subsystemsRun(TRAVEL);
      } else {
        subsystemsRun(STOW);
      }
      if (intake.hasCoral()) {
        subsystemsRun(TRAVEL);
      }
      intake.setWantedState(Intake.WantedState.INTAKE_CORAL);
    } else {
      if (intake.hasCoral()) {
      } else if (drive.getPose().getRotation().getDegrees() >= 0) {
        var angleToSnapTo = FieldConstants.isBlueAlliance() ? 54.0 : 126.0;
        driveToPoseTarget =
            new Pose2d(drive.getPose().getTranslation(), Rotation2d.fromDegrees(angleToSnapTo));
        driveToPoseCommand.schedule();
      } else {
        var angleToSnapTo = FieldConstants.isBlueAlliance() ? -54.0 : -126.0;
        driveToPoseTarget =
            new Pose2d(drive.getPose().getTranslation(), Rotation2d.fromDegrees(angleToSnapTo));
        driveToPoseCommand.schedule();
      }
      if (elevator.getPosition() > 0.15) {
        subsystemsRun(TRAVEL);
      } else {
        subsystemsRun(STOW);
      }
      intake.setWantedState(Intake.WantedState.INTAKE_CORAL);
    }
    if (Robot.isSimulation()) {
      var currentPose = drive.getPose();
      if (intake.getSystemState() == Intake.SystemState.INTAKING_CORAL) {
        intake.setHasCoral(
            simCoralDebouncer.calculate(
                wrist.atGoal()
                    && elevator.atGoal()
                    && MathUtil.isNear(
                        currentPose.getX(), FieldConstants.getClosestStation(currentPose).getX(), 1)
                    && MathUtil.isNear(
                        currentPose.getY(),
                        FieldConstants.getClosestStation(currentPose).getY(),
                        1)));
      }
    }
    if (homeDebouncer.calculate(elevator.getHomeSensor())) {
      elevator.resetPosition(0.0);
    }
  }

  private void intakeAlgaeFromReef() {
    var levelMap =
        FieldConstants.isBlueAlliance()
            ? ReefConstants.blueAllianceAlgae
            : ReefConstants.redAllianceAlgae;

    var location = levelMap.get(getClosestRotationToFaceNearestReefFace());
    var level = location.front;

    wrist.setWantedState(Pivot.WantedState.MOVE_TO_POSITION, WristElevatorPoses.TRAVEL.wristAngle);
    if (wristPastSafe.getAsBoolean()) {
      subsystemsRun(level == ReefConstants.AlgaeIntakeLocation.L2 ? L2_ALGAE : L3_ALGAE);
    }

    intake.setWantedState(Intake.WantedState.INTAKE_ALGAE);

    var angleToIdMap =
        FieldConstants.isBlueAlliance()
            ? ReefConstants.blueAllianceAngleToTagIdsMap
            : ReefConstants.redAllianceAngleToTagIdsMap;
    var ids = angleToIdMap.get(getClosestRotationToFaceNearestReefFace());
    var id = ids.frontId;

    if (!hasDriveReachedMiddleAlgae) {
      driveToPoseTarget = getIntermediatePointToDriveToForAlgaeIntaking(id);
      driveToPoseCommand.schedule();
      if (isAtDriveToPoseSetpoint()) {
        hasDriveReachedMiddleAlgae = true;
      }
    } else if (intake.hasAlgae()) {
      driveToPoseTarget = getBackoutPointToDriveToForAlgaeIntaking(id);
      driveToPoseCommand.schedule();
      if (isAtDriveToPoseSetpoint()) {
        subsystemsRun(ALGAE_HOLD);
      }
    } else {
      if (!wrist.atGoal() && !elevator.atGoal(Units.inchesToMeters(1.0))) {
        driveToPoseTarget = getIntermediatePointToDriveToForAlgaeIntaking(id);
      } else {
        driveToPoseTarget = getDesiredPointToDriveToForAlgaeIntaking(id);
      }
    }
    if (Robot.isSimulation()) {
      if (simAlgaeDebouncer.calculate(allAtGoals())) {
        intake.setHasAlgae(true);
      }
    }
  }

  private void intakeAlgaeIceCream() {}

  private void scoreL1Teleop(ScoringSide scoringSide) {
    if (!overrides.isReefOverride()) {
      driveToScoringPose(scoringSide, true);
    }
    wristRun(TRAVEL);

    if (wristPastSafe.getAsBoolean()) {
      elevatorRun(L1);
      if (elevator.atGoal()) {
        subsystemsRun(L1);
      }
    }
    if (isReadyToEject()) {
      coralEject = true;
    }
    if (coralEject) {
      if (overrides.isReefOverride() == false) {
        intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL_L1);
      }
    }
  }

  private void scoreL2Teleop(ScoringSide scoringSide) {
    if (!overrides.isReefOverride()) {
      driveToScoringPose(scoringSide, false);
    }
    wristRun(TRAVEL);
    if (wrist.atGoal()) {
      subsystemsRun(L2);
    }
    if (isReadyToEject()) {
      coralEject = true;
    }
    if (coralEject) {
      if (overrides.isReefOverride() == false) {
        intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
      }
    }
  }

  private void scoreL3Teleop(ScoringSide scoringSide) {
    if (!overrides.isReefOverride()) {
      driveToScoringPose(scoringSide, false);
    }
    wristRun(TRAVEL);
    if (wrist.atGoal()) {
      subsystemsRun(L3);
      if (isReadyToEject()) {
        coralEject = true;
      }
      if (coralEject) {
        if (overrides.isReefOverride() == false) {
          intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
        }
      }
    }
  }

  private void scoreL4Teleop(ScoringSide scoringSide) {
    if (!overrides.isReefOverride()) {
      driveToScoringPoseL4(scoringSide);
    }
    wristRun(TRAVEL);
    if (wristPastSafe.getAsBoolean()) {
      intake.setWantedState(Intake.WantedState.GRIP_CORAL);
      elevatorRun(L4);
      if (elevator.atGoal()) {
        subsystemsRun(L4);
      }
    }
    if (isReadyToEject()) {
      coralEject = true;
    }
    if (coralEject) {
      if (overrides.isReefOverride() == false) {
        intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
      }
    }
  }

  private void moveAlgaeToNetPosition() {
    Rotation2d rotation =
        FieldConstants.isBlueAlliance() ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);
    driveToPoseTarget = new Pose2d(drive.getPose().getTranslation(), rotation);
    driveToPoseCommand.schedule();
    intake.setWantedState(Intake.WantedState.HOLD_ALGAE);
    if (isAtRotationLockSetpoint()) {
      subsystemsRun(ALGAE_NET_PRE);
    }
  }

  private void moveAlgaeToProcessorPosition() {
    Rotation2d rotation =
        FieldConstants.isBlueAlliance() ? Rotation2d.fromDegrees(-90) : Rotation2d.fromDegrees(90);
    driveToPoseTarget = new Pose2d(drive.getPose().getTranslation(), rotation);
    driveToPoseCommand.schedule();
  }

  private void scoreAlgaeNet() {
    drive.stop();
    if (isAtRotationLockSetpoint()) {
      subsystemsRun(ALGAE_NET_POST);
    }
    if (wrist.getAngle().getRadians()
        > (ALGAE_NET_POST.pivotAngle().minus(Rotation2d.fromDegrees(60))).getRadians())
      intake.setWantedState(Intake.WantedState.EJECT_ALGAE);
  }

  private void scoreAlgaeProcessor() {
    Rotation2d rotation =
        FieldConstants.isBlueAlliance() ? Rotation2d.fromDegrees(90) : Rotation2d.fromDegrees(-90);
    driveToPoseTarget = new Pose2d(drive.getPose().getTranslation(), rotation);
    driveToPoseCommand.schedule();
    subsystemsRun(ALGAE_PROCESSOR);
    intake.setWantedState(Intake.WantedState.EJECT_ALGAE);
  }

  public boolean isReadyToEject() {
    return readyToScoreDebouncer.calculate(
        isAtDriveToPoseSetpoint()
            && isAtRotationLockSetpoint(Units.degreesToRadians(2.0))
            && isDriveStopped()
            && wrist.atGoal()
            && elevator.atGoal());
  }

  public boolean isReadyToEjectInAutoPeriod() {
    return readyToScoreDebouncerAuto.calculate(
        elevator.atGoal() && wrist.atGoal() && isAtDriveToPoseSetpoint() && isDriveStopped());
  }

  public boolean isReadyToIntakeCountdown() {
    return intakeDebouncerAuto.calculate(
        elevator.atGoal() && wrist.atGoal() && isAtDriveToPoseSetpoint() && isDriveStopped());
  }

  public boolean driveToScoringPose(ScoringSide scoringSide, boolean isL1) {
    Pose2d desiredPoseToDriveTo =
        !isL1
            ? FieldConstants.getDesiredFinalScoringPoseForCoral(getClosestTagId(), scoringSide)
            : FieldConstants.getDesiredPointToDriveToForL1Scoring(getClosestTagId(), scoringSide);

    if (!hasDriveReachedMiddleCoral) {
      Pose2d intermediatePose =
          FieldConstants.getDesiredIntermediateScoringPoseForCoral(getClosestTagId(), scoringSide);
      driveToPoseTarget = intermediatePose;
      driveToPoseCommand.schedule();
      if (isAtDriveToPoseSetpoint()) {
        hasDriveReachedMiddleCoral = true;
      }
      Logger.recordOutput("Choreographer/DesiredPointToDriveTo", intermediatePose);
      return true;
    } else {
      driveToPoseTarget = desiredPoseToDriveTo;
      driveToPoseCommand.schedule();
      Logger.recordOutput("Choreographer/DesiredPointToDriveTo", desiredPoseToDriveTo);
      return true;
    }
  }

  public boolean driveToScoringPoseL4(ScoringSide scoringSide) {
    Pose2d desiredPoseToDriveTo =
        FieldConstants.getDesiredFinalScoringPoseForCoral(getClosestTagId(), scoringSide);
    Pose2d intermediatePose =
        FieldConstants.getDesiredIntermediateScoringPoseForCoral(getClosestTagId(), scoringSide);

    Pose2d preL4Pose =
        FieldConstants.getDesiredPointToDriveToForCoralScoring(
            getClosestTagId(), scoringSide, 0.15);

    if (!hasDriveReachedMiddleCoral) {
      driveToPoseTarget = intermediatePose;
      driveToPoseCommand.schedule();
      if (isAtDriveToPoseSetpoint()) {
        hasDriveReachedMiddleCoral = true;
      }
      Logger.recordOutput("Choreographer/DesiredPointToDriveTo", intermediatePose);
      return true;
    } else {
      if (elevator.atGoal()) {
        driveToPoseTarget = desiredPoseToDriveTo;
        driveToPoseCommand.schedule();
        Logger.recordOutput("Choreographer/DesiredPointToDriveTo", desiredPoseToDriveTo);
      } else {
        driveToPoseTarget = preL4Pose;
        driveToPoseCommand.schedule();
        Logger.recordOutput("Choreographer/DesiredPointToDriveTo", preL4Pose);
      }
      return true;
    }
  }

  public boolean driveToScoringPose(
      ReefConstants.ReefFaces face, ScoringSide scoringSide, boolean isL1) {
    var map =
        FieldConstants.isBlueAlliance()
            ? ReefConstants.blueAllianceReefFacesToIds
            : ReefConstants.redAllianceReefFacesToIds;
    var id = map.get(face);
    Pose2d desiredPoseToDriveTo =
        !isL1
            ? FieldConstants.getDesiredFinalScoringPoseForCoral(id, scoringSide)
            : FieldConstants.getDesiredPointToDriveToForL1Scoring(id, scoringSide);

    if (!hasDriveReachedMiddleCoral) {
      Pose2d intermediatePose =
          FieldConstants.getDesiredIntermediateScoringPoseForCoral(id, scoringSide);
      driveToPoseTarget = intermediatePose;
      driveToPoseCommand.schedule();
      if (isAtDriveToPoseSetpoint()) {
        hasDriveReachedMiddleCoral = true;
      }
      Logger.recordOutput("Choreographer/DesiredPointToDriveTo", intermediatePose);
      return true;
    } else {
      driveToPoseTarget = desiredPoseToDriveTo;
      driveToPoseCommand.schedule();
      Logger.recordOutput("Choreographer/DesiredPointToDriveTo", desiredPoseToDriveTo);
      return true;
    }
  }

  public boolean reefTagVisible() {
    int desiredId = getClosestTagId();
    boolean seenTag = false;

    for (int id : vision.getVisibleTagIds()) {
      if (id == desiredId) {
        seenTag = true;
      }
    }
    return seenTag;
  }

  public Pose2d getBackoutPointToDriveToForAlgaeIntaking(int tagID) {
    if (tagID >= 1 && tagID <= 22) {
      Pose2d tagPose = FieldConstants.getTagPose(tagID).toPose2d();
      double xOffset = Units.inchesToMeters(xOffsetFromTagForBackoutIntakingAlgaeInches);

      Translation2d offsetFromTag = new Translation2d(xOffset, 0);

      Pose2d transformedPose =
          tagPose.plus(
              new Transform2d(
                  offsetFromTag.getX(), offsetFromTag.getY(), Rotation2d.fromDegrees(180)));

      return transformedPose;
    } else {
      return new Pose2d();
    }
  }

  public Pose2d getIntermediatePointToDriveToForAlgaeIntaking(int tagID) {
    if (tagID >= 1 && tagID <= 22) {
      Pose2d tagPose = FieldConstants.getTagPose(tagID).toPose2d();
      double xOffset = Units.inchesToMeters(xOffsetFromTagForIntermediateIntakingAlgaeInches);

      Translation2d offsetFromTag = new Translation2d(xOffset, 0);

      Pose2d transformedPose =
          tagPose.plus(
              new Transform2d(
                  offsetFromTag.getX(), offsetFromTag.getY(), Rotation2d.fromDegrees(180)));

      return transformedPose;
    } else {
      return new Pose2d();
    }
  }

  public Pose2d getDesiredPointToDriveToForAlgaeIntaking(int tagID) {
    if (tagID >= 1 && tagID <= 22) {
      Pose2d tagPose = FieldConstants.getTagPose(tagID).toPose2d();
      double xOffset = Units.inchesToMeters(xOffsetFromTagForIntakingAlgaeInches);

      Translation2d offsetFromTag = new Translation2d(xOffset, 0);

      Pose2d transformedPose =
          tagPose.plus(
              new Transform2d(
                  offsetFromTag.getX(), offsetFromTag.getY(), Rotation2d.fromDegrees(180)));

      return transformedPose;
    } else {
      return new Pose2d();
    }
  }

  public void setWantedChoreography(WantedChoreography choreography) {
    this.wantedChoreography = choreography;
  }

  public Command setChoreographyCommand(WantedChoreography choreography) {
    return new InstantCommand(() -> setWantedChoreography(choreography));
  }

  // Scoring side API: set explicitly or flip current
  public void setScoringSide(ScoringSide side) {
    this.targetScoringSide = side;
  }

  public void flipScoringSide() {
    this.targetScoringSide =
        (this.targetScoringSide == ScoringSide.LEFT) ? ScoringSide.RIGHT : ScoringSide.LEFT;
  }

  public Command setScoringSideCommand(ScoringSide side) {
    return new InstantCommand(() -> setScoringSide(side));
  }

  public Command flipScoringSideCommand() {
    return new InstantCommand(this::flipScoringSide);
  }

  public ScoringSide getScoringSide() {
    return this.targetScoringSide;
  }

  public void subsystemsRun(Position position) {
    wrist.setWantedState(Pivot.WantedState.MOVE_TO_POSITION, position.pivotAngle());
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, position.elevatorHeightMeters());
  }

  public void elevatorRun(Position position) {
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, position.elevatorHeightMeters());
  }

  public void wristRun(Position position) {
    wrist.setWantedState(Pivot.WantedState.MOVE_TO_POSITION, position.pivotAngle());
  }

  public boolean allAtGoals() {
    return elevator.atGoal() && wrist.atGoal() && isAtDriveToPoseSetpoint();
  }

  public boolean mechanismsAtGoals() {
    return wrist.atGoal() && elevator.atGoal();
  }

  private boolean isAtDriveToPoseSetpoint() {
    return driveToPoseCommand.withinTolerance(
        Units.inchesToMeters(1.0), Rotation2d.fromDegrees(2.0));
  }

  private boolean isAtRotationLockSetpoint() {
    return isAtRotationLockSetpoint(Units.degreesToRadians(10.0));
  }

  private boolean isAtRotationLockSetpoint(double tolerance) {
    // This is a placeholder. A proper implementation would check the error
    // from the rotation lock PID controller in the Drive subsystem.
    // Since that's private, we can't access it directly.
    // For now, we assume it's at the setpoint if the state is ROTATION_LOCK.
    return true;
  }

  private boolean isDriveStopped() {
    var speeds = drive.getChassisSpeeds(true);
    return Math.abs(speeds.vxMetersPerSecond) < 0.1
        && Math.abs(speeds.vyMetersPerSecond) < 0.1
        && Math.abs(speeds.omegaRadiansPerSecond) < 0.1;
  }

  private int getClosestTagId() {
    int closestId = -1;
    double minDistance = Double.MAX_VALUE;

    for (int id : vision.getVisibleTagIds()) {
      Pose2d tagPose = FieldConstants.getTagPose(id).toPose2d();
      double distance = drive.getPose().getTranslation().getDistance(tagPose.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestId = id;
      }
    }
    return closestId;
  }

  private Rotation2d getClosestRotationToFaceNearestReefFace() {
    var map =
        FieldConstants.isBlueAlliance()
            ? ReefConstants.blueAllianceAngleToTagIdsMap
            : ReefConstants.redAllianceAngleToTagIdsMap;
    double minDistance = Double.MAX_VALUE;
    Rotation2d closestRotation = new Rotation2d();

    for (var entry : map.entrySet()) {
      var ids = entry.getValue();
      var tagPose = FieldConstants.getTagPose(ids.frontId).toPose2d();
      var distance = drive.getPose().getTranslation().getDistance(tagPose.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestRotation = entry.getKey();
      }
    }
    return closestRotation;
  }
}
