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
import frc.robot.Robot;
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

  private static final double defaultTeleopTranslationCoefficient = 1.0;

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

  public Choreographer(
      Drive drive,
      Intake intake,
      Elevator elevator,
      Pivot wrist,
      OverridePublisher overrides,
      Vision vision) {
    this.drive = drive;
    this.intake = intake;
    this.elevator = elevator;
    this.wrist = wrist;
    this.overrides = overrides;
    this.vision = vision;

    this.wristPastSafe = () -> wrist.getAngle().getRadians() > .6;
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
      System.out.println("⚠️ CHOREOGRAPHER DISABLED - Subsystems can be controlled directly");
    } else {
      System.out.println("✅ CHOREOGRAPHER ENABLED - Normal operation resumed");
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

    drive.setWantedState(Drive.WantedState.TELEOP_DRIVE);
    drive.setTeleopVelocityCoefficient(defaultTeleopTranslationCoefficient);
    drive.setRotationVelocityCoefficient(1.0);

    intake.setWantedState(
        wrist.atGoal() && elevator.atGoal()
            ? Intake.WantedState.HOLD_ALGAE
            : Intake.WantedState.HOLD_ALGAE_HARDER);
  }

  private void holdingCoral() {
    coralEject = false;
    subsystemsRun(TRAVEL);
    intake.setWantedState(Intake.WantedState.OFF);
    drive.setWantedState(Drive.WantedState.TELEOP_DRIVE);
    drive.setTeleopVelocityCoefficient(defaultTeleopTranslationCoefficient);
    drive.setRotationVelocityCoefficient(1.0);
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
    drive.setWantedState(Drive.WantedState.TELEOP_DRIVE);
    drive.setTeleopVelocityCoefficient(defaultTeleopTranslationCoefficient);
    drive.setRotationVelocityCoefficient(1.0);
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
        drive.setState(Drive.WantedState.TELEOP_DRIVE);
      } else if (drive.getPose().getRotation().getDegrees() >= 0) {
        var angleToSnapTo = FieldConstants.isBlueAlliance() ? 54.0 : 126.0;
        drive.setTargetRotation(Rotation2d.fromDegrees(angleToSnapTo));
      } else {
        var angleToSnapTo = FieldConstants.isBlueAlliance() ? -54.0 : -126.0;
        drive.setTargetRotation(Rotation2d.fromDegrees(angleToSnapTo));
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

    var location = levelMap.get(drive.getClosestRotationToFaceNearestReefFace());
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
    var ids = angleToIdMap.get(drive.getClosestRotationToFaceNearestReefFace());
    var id = ids.frontId;

    if (!hasDriveReachedMiddleAlgae) {
      drive.setDesiredPoseForDriveToPoint(getIntermediatePointToDriveToForAlgaeIntaking(id));
      if (drive.isAtDriveToPointSetpoint()) {
        hasDriveReachedMiddleAlgae = true;
      }
    } else if (intake.hasAlgae()) {
      drive.setDesiredPoseForDriveToPoint(getBackoutPointToDriveToForAlgaeIntaking(id));
      if (drive.isAtDriveToPointSetpoint()) {
        subsystemsRun(ALGAE_HOLD);
      }
    } else {
      if (!wrist.atGoal() && !elevator.atGoal(Units.inchesToMeters(1.0))) {
        drive.setDesiredPoseForDriveToPoint(getIntermediatePointToDriveToForAlgaeIntaking(id));
      } else {
        drive.setDesiredPoseForDriveToPoint(getDesiredPointToDriveToForAlgaeIntaking(id));
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
    Rotation2d rotation = FieldConstants.isBlueAlliance() ? Rotation2d.k180deg : Rotation2d.kZero;
    /*if (Math.abs(
            drive
                .getRobotPoseFromSwerveDriveOdometry()
                .getRotation()
                .getDegrees())
        > 90) {
      rotation = Rotation2d.k180deg;
    } else {
      rotation = Rotation2d.kZero;
    }*/
    drive.setTargetRotation(rotation);

    intake.setWantedState(Intake.WantedState.HOLD_ALGAE);
    drive.setTeleopVelocityCoefficient(0.4);

    if (drive.isAtDesiredRotation()) {
      subsystemsRun(ALGAE_NET_PRE);
    }
  }

  private void moveAlgaeToProcessorPosition() {
    Rotation2d rotation =
        FieldConstants.isOnBlueAlliance(drive.getPose())
            ? Rotation2d.kCW_90deg
            : Rotation2d.kCCW_90deg;
    drive.setTargetRotation(rotation);
  }

  private void scoreAlgaeNet() {
    drive.setTeleopVelocityCoefficient(0.0);
    if (drive.isAtDesiredRotation()) {
      subsystemsRun(ALGAE_NET_POST);
    }
    if (wrist.getAngle().getRadians()
        > (ALGAE_NET_POST.pivotAngle().minus(Rotation2d.fromDegrees(60))).getRadians())
      intake.setWantedState(Intake.WantedState.EJECT_ALGAE);
  }

  private void scoreAlgaeProcessor() {
    Rotation2d rotation =
        FieldConstants.isBlueAlliance() ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg;
    if (drive.getPose().getRotation().getDegrees() > 0) {
      rotation = Rotation2d.kCCW_90deg;
    } else {
      rotation = Rotation2d.kCW_90deg;
    }
    drive.setTargetRotation(rotation);
    subsystemsRun(ALGAE_PROCESSOR);
    intake.setWantedState(Intake.WantedState.EJECT_ALGAE);
  }

  public boolean isReadyToEject() {
    return readyToScoreDebouncer.calculate(
        drive.isAtDriveToPointSetpoint()
            && drive.isAtDesiredRotation(Units.degreesToRadians(2.0))
            && drive.isStopped()
            && wrist.atGoal()
            && elevator.atGoal());
  }

  public boolean isReadyToEjectInAutoPeriod() {
    return readyToScoreDebouncerAuto.calculate(
        elevator.atGoal()
            && wrist.atGoal()
            && drive.isAtEndOfChoreoTrajectoryOrDriveToPoint()
            && drive.isStopped());
  }

  public boolean isReadyToIntakeCountdown() {
    return intakeDebouncerAuto.calculate(
        elevator.atGoal()
            && wrist.atGoal()
            && drive.isAtEndOfChoreoTrajectoryOrDriveToPoint()
            && drive.isStopped());
  }

  public boolean driveToScoringPose(ScoringSide scoringSide, boolean isL1) {
    Pose2d desiredPoseToDriveTo =
        !isL1
            ? FieldConstants.getDesiredFinalScoringPoseForCoral(
                drive.getClosestTagId(), scoringSide)
            : FieldConstants.getDesiredPointToDriveToForL1Scoring(
                drive.getClosestTagId(), scoringSide);

    if (!hasDriveReachedMiddleCoral) {
      Pose2d intermediatePose =
          FieldConstants.getDesiredIntermediateScoringPoseForCoral(
              drive.getClosestTagId(), scoringSide);
      drive.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(intermediatePose, 3.0);
      if (drive.isAtDriveToPointSetpoint()) {
        hasDriveReachedMiddleCoral = true;
      }
      Logger.recordOutput("Choreographer/DesiredPointToDriveTo", intermediatePose);
      return true;
    } else {
      drive.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(desiredPoseToDriveTo, 3.0);
      Logger.recordOutput("Choreographer/DesiredPointToDriveTo", desiredPoseToDriveTo);
      return true;
    }
  }

  public boolean driveToScoringPoseL4(ScoringSide scoringSide) {
    Pose2d desiredPoseToDriveTo =
        FieldConstants.getDesiredFinalScoringPoseForCoral(drive.getClosestTagId(), scoringSide);
    Pose2d intermediatePose =
        FieldConstants.getDesiredIntermediateScoringPoseForCoral(
            drive.getClosestTagId(), scoringSide);

    Pose2d preL4Pose =
        FieldConstants.getDesiredPointToDriveToForCoralScoring(
            drive.getClosestTagId(), scoringSide, 0.15);

    if (!hasDriveReachedMiddleCoral) {
      drive.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(intermediatePose, 3.0);
      if (drive.isAtDriveToPointSetpoint()) {
        hasDriveReachedMiddleCoral = true;
      }
      Logger.recordOutput("Choreographer/DesiredPointToDriveTo", intermediatePose);
      return true;
    } else {
      if (elevator.atGoal()) {
        drive.setDesiredPoseForDriveToPointWithConstraints(desiredPoseToDriveTo, 0.5, 3.0);
        Logger.recordOutput("Choreographer/DesiredPointToDriveTo", desiredPoseToDriveTo);
      } else {
        drive.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(preL4Pose, 3.0);
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
      drive.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(intermediatePose, 3.0);
      if (drive.isAtDriveToPointSetpoint()) {
        hasDriveReachedMiddleCoral = true;
      }
      Logger.recordOutput("Choreographer/DesiredPointToDriveTo", intermediatePose);
      return true;
    } else {
      drive.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(desiredPoseToDriveTo, 3.0);
      Logger.recordOutput("Choreographer/DesiredPointToDriveTo", desiredPoseToDriveTo);
      return true;
    }
  }

  public boolean reefTagVisible() {
    int desiredId = drive.getClosestTagId();
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
              new Transform2d(offsetFromTag.getX(), offsetFromTag.getY(), Rotation2d.k180deg));

      return transformedPose;
    } else {
      return Pose2d.kZero;
    }
  }

  public Pose2d getIntermediatePointToDriveToForAlgaeIntaking(int tagID) {
    if (tagID >= 1 && tagID <= 22) {
      Pose2d tagPose = FieldConstants.getTagPose(tagID).toPose2d();
      double xOffset = Units.inchesToMeters(xOffsetFromTagForIntermediateIntakingAlgaeInches);

      Translation2d offsetFromTag = new Translation2d(xOffset, 0);

      Pose2d transformedPose =
          tagPose.plus(
              new Transform2d(offsetFromTag.getX(), offsetFromTag.getY(), Rotation2d.k180deg));

      return transformedPose;
    } else {
      return Pose2d.kZero;
    }
  }

  public Pose2d getDesiredPointToDriveToForAlgaeIntaking(int tagID) {
    if (tagID >= 1 && tagID <= 22) {
      Pose2d tagPose = FieldConstants.getTagPose(tagID).toPose2d();
      double xOffset = Units.inchesToMeters(xOffsetFromTagForIntakingAlgaeInches);

      Translation2d offsetFromTag = new Translation2d(xOffset, 0);

      Pose2d transformedPose =
          tagPose.plus(
              new Transform2d(offsetFromTag.getX(), offsetFromTag.getY(), Rotation2d.k180deg));

      return transformedPose;
    } else {
      return Pose2d.kZero;
    }
  }

  public void setWantedChoreography(WantedChoreography choreography) {
    this.wantedChoreography = choreography;
  }

  public Command setChoreographyCommand(WantedChoreography choreography) {
    Command commandToReturn = new InstantCommand(() -> setWantedChoreography(choreography));
    return commandToReturn;
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
    return elevator.atGoal() && wrist.atGoal() && drive.isAtDriveToPointSetpoint();
  }

  public boolean mechanismsAtGoals() {
    return wrist.atGoal() && elevator.atGoal();
  }
}
