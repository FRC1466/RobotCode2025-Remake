// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.ChoreographerConstants.*;
import static frc.robot.constants.ChoreographerPositions.*;

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
import frc.robot.RobotState;
import frc.robot.constants.ChoreographerConstants.ScoringDirection;
import frc.robot.constants.ChoreographerConstants.ScoringSide;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ReefConstants;
import frc.robot.subsystems.algaeSlapdown.AlgaeSlapdown;
import frc.robot.subsystems.coralSlapdown.CoralSlapdown;
import frc.robot.subsystems.diffwrist.DifferentialWristPivot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.overridePublisher.OverridePublisher;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Position;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Choreographer extends SubsystemBase {
  private final Drive drive;
  private final Intake intake;
  private final Elevator elevator;
  private final CoralSlapdown coralSlapdown;
  private final AlgaeSlapdown algaeSlapdown;
  private final Pivot pivot;
  private final DifferentialWristPivot differential;
  private final OverridePublisher overrides;
  private final Vision vision;

  private static final double defaultTeleopTranslationCoefficient = 1.0;

  // Debouncers
  private final Debouncer simCoralDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kRising);
  private final Debouncer simAlgaeDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kRising);
  private final Debouncer readyToScoreDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kRising);
  private final Debouncer readyToScoreDebouncerAuto =
      new Debouncer(0.5, Debouncer.DebounceType.kRising);
  private final Debouncer intakeDebouncerAuto = new Debouncer(0.1, Debouncer.DebounceType.kRising);

  public enum WantedCoralLocation {
    CLAW,
    SLAPDOWN
  }

  public enum CurrentCoralLocation {
    CLAW,
    SLAPDOWN,
    NONE
  }

  public enum WantedChoreography {
    HOME,
    STOPPED,
    DEFAULT_STATE,
    INTAKE_CORAL_FROM_GROUND,
    SCORE_L1,
    SCORE_L2,
    SCORE_L3,
    SCORE_L4,
    INTAKE_ALGAE_REEF,
    INTAKE_ALGAE_GROUND,
    INTAKE_ALGAE_ICE_CREAM,
    MOVE_ALGAE_TO_SAFE,
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
    INTAKE_CORAL_FROM_GROUND,
    SCORE_L1,
    SCORE_L2,
    SCORE_L3,
    SCORE_L4,
    INTAKE_ALGAE_REEF,
    INTAKE_ALGAE_GROUND,
    INTAKE_ALGAE_ICE_CREAM,
    MOVE_ALGAE_TO_SAFE,
    MOVE_ALGAE_TO_NET_POSITION,
    SCORE_ALGAE_IN_NET,
    MOVE_ALGAE_TO_PROCESSOR_POSITION,
    SCORE_ALGAE_IN_PROCESSOR,
    EJECT_ALGAE,
    EJECT_CORAL
  }

  private WantedChoreography wantedChoreography = WantedChoreography.STOPPED;
  private CurrentChoreography currentChoreography = CurrentChoreography.STOPPED;
  private CurrentChoreography previousChoreography;

  @Setter private WantedCoralLocation wantedCoralLocation = WantedCoralLocation.CLAW;

  private ScoringDirection scoringDirection = ScoringDirection.BACK;
  private ScoringSide targetScoringSide = ScoringSide.LEFT;

  private boolean hasDriveReachedIntermediatePoseForReefAlgaePickup = false;

  private boolean coralEject = false;

  public Choreographer(
      Drive drive,
      Intake intake,
      Elevator elevator,
      Pivot pivot,
      CoralSlapdown coralSlapdown,
      AlgaeSlapdown algaeSlapdown,
      DifferentialWristPivot differentialWristPivot,
      OverridePublisher overrides,
      Vision vision) {
    this.drive = drive;
    this.intake = intake;
    this.elevator = elevator;
    this.pivot = pivot;
    this.coralSlapdown = coralSlapdown;
    this.algaeSlapdown = algaeSlapdown;
    this.differential = differentialWristPivot;
    this.overrides = overrides;
    this.vision = vision;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Choreographer/Wanted", wantedChoreography);
    Logger.recordOutput("Choreographer/Current", currentChoreography);
    Logger.recordOutput("Choreographer/Previous", previousChoreography);

    Logger.recordOutput("Choreographer/WantedCoralLocation", wantedCoralLocation);
    Logger.recordOutput("Choreographer/TargetScoringSide", targetScoringSide);

    currentChoreography = computeChoreography();
    applyChoreography();
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
      case INTAKE_CORAL_FROM_GROUND:
        currentChoreography = CurrentChoreography.INTAKE_CORAL_FROM_GROUND;
        break;
      case DEFAULT_STATE:
        if (intake.hasCoralClaw() || intake.hasCoralSlapdown()) {
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
      case INTAKE_ALGAE_GROUND:
        currentChoreography = CurrentChoreography.INTAKE_ALGAE_GROUND;
        break;
      case MOVE_ALGAE_TO_SAFE:
        currentChoreography = CurrentChoreography.MOVE_ALGAE_TO_SAFE;
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
    scoringDirection = RobotState.getInstance().getFacingSideRelativeToClosestTag();
    Logger.recordOutput("Choreographer/ScoringDirection", scoringDirection);
    if (previousChoreography != currentChoreography) {
      resetDebouncers();
      hasDriveReachedIntermediatePoseForReefAlgaePickup = false;
    }

    switch (currentChoreography) {
      case HOME:
        home();
        break;
      case INTAKE_CORAL_FROM_GROUND:
        intakeCoralFromGround();
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
        if (DriverStation.isAutonomous()) {
          scoreL1Auto(targetScoringSide);
        } else {
          scoreL1Teleop(targetScoringSide);
        }
        break;
      case SCORE_L2:
        if (DriverStation.isAutonomous()) {
          scoreL2Auto(targetScoringSide);
        } else {
          scoreL2Teleop(targetScoringSide);
        }
        break;
      case SCORE_L3:
        if (DriverStation.isAutonomous()) {
          scoreL3Auto(targetScoringSide);
        } else {
          scoreL3Teleop(targetScoringSide);
        }
        break;
      case SCORE_L4:
        if (DriverStation.isAutonomous()) {
          scoreL4Auto(targetScoringSide);
        } else {
          scoreL4Teleop(targetScoringSide);
        }
        break;
      case INTAKE_ALGAE_ICE_CREAM:
        intakeAlgaeIceCream();
        break;
      case INTAKE_ALGAE_REEF:
        intakeAlgaeFromReef();
        break;
      case INTAKE_ALGAE_GROUND:
        intakeAlgaeFromGround();
        break;
      case MOVE_ALGAE_TO_SAFE:
        moveAlgaeToSafePosition();
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

  private void resetDebouncers() {
    simAlgaeDebouncer.calculate(false);
    simCoralDebouncer.calculate(false);
    readyToScoreDebouncer.calculate(false);
    readyToScoreDebouncerAuto.calculate(false);
    intakeDebouncerAuto.calculate(false);
  }

  private void home() {}

  private void stopped() {
    pivot.setWantedState(Pivot.WantedState.IDLE);
    elevator.setWantedState(Elevator.WantedState.IDLE);
    coralSlapdown.setWantedState(CoralSlapdown.WantedState.IDLE);
    algaeSlapdown.setWantedState(AlgaeSlapdown.WantedState.IDLE);
    differential.setWantedState(DifferentialWristPivot.WantedState.IDLE);
    intake.setWantedState(Intake.WantedState.OFF);
  }

  private void holdingAlgae() {
    subsystemsRunAlgaeSlapdownLater(ALGAE_STOW);

    drive.setWantedState(Drive.WantedState.TELEOP_DRIVE);
    drive.setTeleopVelocityCoefficient(defaultTeleopTranslationCoefficient);
    drive.setRotationVelocityCoefficient(1.0);

    intake.setWantedState(Intake.WantedState.HOLD_ALGAE);
  }

  private void holdingCoral() {
    coralEject = false;
    if (wantedCoralLocation == WantedCoralLocation.SLAPDOWN) {
      subsystemsRun(CORAL_STOW_L1);
    } else {
      subsystemsRun(CORAL_STOW);
    }
    intake.setWantedState(Intake.WantedState.HOLD_CORAL);
    drive.setWantedState(Drive.WantedState.TELEOP_DRIVE);
    drive.setTeleopVelocityCoefficient(defaultTeleopTranslationCoefficient);
    drive.setRotationVelocityCoefficient(1.0);
    handleCoralLocationChoreography();
  }

  private void holdingCoralAuto() {
    coralEject = false;
    subsystemsRun(CORAL_STOW);
    intake.setWantedState(Intake.WantedState.HOLD_CORAL);
    handleCoralLocationChoreography();
  }

  private void noPiece() {
    subsystemsRun(STOW);
    intake.setWantedState(Intake.WantedState.OFF);
    drive.setWantedState(Drive.WantedState.TELEOP_DRIVE);
    drive.setTeleopVelocityCoefficient(defaultTeleopTranslationCoefficient);
    drive.setRotationVelocityCoefficient(1.0);
  }

  private void noPieceAuto() {
    subsystemsRun(STOW);
    intake.setWantedState(Intake.WantedState.OFF);
  }

  private void intakeCoralFromGround() {
    subsystemsRun(CORAL_INTAKE);
    intake.setWantedState(Intake.WantedState.INTAKE_CORAL);
    if (simCoralDebouncer.calculate(mechanismsAtGoals())) {
      intake.setHasCoralSlapdown(true);
    }
  }

  private void intakeAlgaeFromReef() {
    var levelMap =
        FieldConstants.isBlueAlliance()
            ? ReefConstants.blueAllianceAlgae
            : ReefConstants.redAllianceAlgae;

    var location = levelMap.get(RobotState.getInstance().getClosestRotationToFaceNearestReefFace());
    var level = location.front;

    subsystemsRun(
        level == ReefConstants.AlgaeIntakeLocation.L2
            ? scoringDirection == ScoringDirection.FRONT
                ? ALGAE_FRONT_INTAKE_L2
                : ALGAE_BACK_INTAKE_L2
            : scoringDirection == ScoringDirection.FRONT
                ? ALGAE_FRONT_INTAKE_L3
                : ALGAE_BACK_INTAKE_L3);

    intake.setWantedState(Intake.WantedState.INTAKE_ALGAE);

    var angleToIdMap =
        FieldConstants.isBlueAlliance()
            ? ReefConstants.blueAllianceAngleToTagIdsMap
            : ReefConstants.redAllianceAngleToTagIdsMap;
    var ids = angleToIdMap.get(RobotState.getInstance().getClosestRotationToFaceNearestReefFace());
    var id = ids.frontId;

    if (!hasDriveReachedIntermediatePoseForReefAlgaePickup) {
      drive.setDesiredPoseForDriveToPoint(getIntermediatePointToDriveToForAlgaeIntaking(id));
      if (drive.isAtDriveToPointSetpoint()) {
        hasDriveReachedIntermediatePoseForReefAlgaePickup = true;
      }
    } else if (intake.hasAlgae()) {
      drive.setDesiredPoseForDriveToPoint(getBackoutPointToDriveToForAlgaeIntaking(id));
      if (drive.isAtDriveToPointSetpoint()) {
        subsystemsRun(ALGAE_STOW);
      }
    } else {
      if (!elevator.atGoal(Units.inchesToMeters(1.0))) {
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

  private void intakeAlgaeFromGround() {
    if (intake.hasAlgae()) {
      intake.setWantedState(Intake.WantedState.HOLD_ALGAE);
      setWantedChoreography(WantedChoreography.MOVE_ALGAE_TO_SAFE);
      return;
    }
    subsystemsRun(ALGAE_STOW);
    algaeSlapdownRun(ALGAE_GROUND_INTAKE);
    intake.setWantedState(Intake.WantedState.INTAKE_ALGAE);
    if (algaeSlapdown.atGoal(Units.degreesToRadians(40))) {
      subsystemsRun(ALGAE_GROUND_INTAKE);
    }
    if (Robot.isSimulation()) {
      if (simAlgaeDebouncer.calculate(mechanismsAtGoals())) {
        intake.setHasAlgae(true);
      }
    }
  }

  private void intakeAlgaeIceCream() {}

  private void moveAlgaeToSafePosition() {
    if (pivot.getAngle().getDegrees() > 45) {
      subsystemsRun(ALGAE_STOW_SAFE);
    } else {
      setWantedChoreography(WantedChoreography.DEFAULT_STATE);
    }
  }

  private void scoreL1Teleop(ScoringSide scoringSide) {
    setWantedCoralLocation(WantedCoralLocation.SLAPDOWN);
    if (intake.hasCoralSlapdown()) {
      if (!overrides.isReefOverride()) {
        driveToScoringPose(scoringSide, scoringDirection, 1);
      }
      subsystemsRun(L1);
      if (isReadyToEject()) {
        coralEject = true;
      }
      if (coralEject) {
        intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
        subsystemsRun(L1_FINAL);
      }
    } else {
      handleCoralLocationChoreography();
    }
  }

  private void scoreL1Auto(ScoringSide scoringSide) {
    subsystemsRun(L1);

    if (isReadyToEjectInAutoPeriod()) {
      coralEject = true;
    }

    if (coralEject) {
      intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
    }
  }

  private void scoreL2Teleop(ScoringSide scoringSide) {
    setWantedCoralLocation(WantedCoralLocation.CLAW);
    if (intake.hasCoralClaw()) {
      if (!overrides.isReefOverride()) {
        driveToScoringPose(scoringSide, scoringDirection, 2);
      }
      if (scoringDirection == ScoringDirection.FRONT) {
        subsystemsRun(L2_FRONT_SCORE);
      } else {
        subsystemsRun(L2_BACK_SCORE);
      }

      if (isReadyToEject()) {
        coralEject = true;
      }
      if (coralEject) {
        if (overrides.isReefOverride() == false) {
          intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
        }
      }
    } else {
      handleCoralLocationChoreography();
    }
  }

  private void scoreL2Auto(ScoringSide scoringSide) {
    if (scoringDirection == ScoringDirection.FRONT) {
      subsystemsRun(L2_FRONT_SCORE);
    } else {
      subsystemsRun(L2_BACK_SCORE);
    }

    if (isReadyToEjectInAutoPeriod()) {
      coralEject = true;
    }

    if (coralEject) {
      intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
    }
  }

  private void scoreL3Teleop(ScoringSide scoringSide) {
    setWantedCoralLocation(WantedCoralLocation.CLAW);
    if (intake.hasCoralClaw()) {
      if (!overrides.isReefOverride()) {
        driveToScoringPose(scoringSide, scoringDirection, 3);
      }
      if (scoringDirection == ScoringDirection.FRONT) {
        subsystemsRun(L3_FRONT_SCORE);
      } else {
        subsystemsRun(L3_BACK_SCORE);
      }
      if (isReadyToEject()) {
        coralEject = true;
      }
      if (coralEject) {
        if (overrides.isReefOverride() == false) {
          intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
        }
      }
    } else {
      handleCoralLocationChoreography();
    }
  }

  private void scoreL3Auto(ScoringSide scoringSide) {
    if (scoringDirection == ScoringDirection.FRONT) {
      subsystemsRun(L3_FRONT_SCORE);
    } else {
      subsystemsRun(L3_BACK_SCORE);
    }

    if (isReadyToEjectInAutoPeriod()) {
      coralEject = true;
    }

    if (coralEject) {
      intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
    }
  }

  private void scoreL4Teleop(ScoringSide scoringSide) {
    setWantedCoralLocation(WantedCoralLocation.CLAW);
    if (intake.hasCoralClaw()) {
      if (!overrides.isReefOverride()) {
        driveToScoringPose(scoringSide, scoringDirection, 4);
      }
      if (scoringDirection == ScoringDirection.FRONT) {
        subsystemsRun(L4_FRONT_SCORE);
      } else {
        subsystemsRun(L4_BACK_SCORE);
      }
      if (isReadyToEject()) {
        coralEject = true;
      }
      if (coralEject) {
        if (overrides.isReefOverride() == false) {
          intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
        }
      }
    } else {
      handleCoralLocationChoreography();
    }
  }

  private void scoreL4Auto(ScoringSide scoringSide) {
    if (scoringDirection == ScoringDirection.FRONT) {
      subsystemsRun(L4_FRONT_SCORE);
    } else {
      subsystemsRun(L4_BACK_SCORE);
    }

    if (isReadyToEjectInAutoPeriod()) {
      coralEject = true;
    }

    if (coralEject) {
      intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
    }
  }

  private void moveAlgaeToNetPosition() {
    Rotation2d rotation = FieldConstants.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg;
    /*if (Math.abs(
            RobotState.getInstance()
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
      if (rotation == Rotation2d.kZero) {
        subsystemsRun(ALGAE_FRONT_BARGE);
      } else {
        subsystemsRun(ALGAE_BACK_BARGE);
      }
    }
  }

  private void moveAlgaeToProcessorPosition() {
    Rotation2d rotation =
        FieldConstants.isBlueAlliance() ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg;
    if (RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation().getDegrees()
        < 0) {
      rotation = Rotation2d.kCW_90deg;
    } else {
      rotation = Rotation2d.kCCW_90deg;
    }
    drive.setTargetRotation(rotation);
    if (drive.isAtDesiredRotation()) {
      subsystemsRun(ALGAE_FRONT_PROCESSOR);
    }
  }

  private void scoreAlgaeNet() {
    drive.setTeleopVelocityCoefficient(0.0);
    Rotation2d rotation = FieldConstants.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg;
    /*if (Math.abs(
            RobotState.getInstance()
                .getRobotPoseFromSwerveDriveOdometry()
                .getRotation()
                .getDegrees())
        > 90) {
      rotation = Rotation2d.k180deg;
    } else {
      rotation = Rotation2d.kZero;
    }*/
    if (drive.isAtDesiredRotation()) {
      if (rotation == Rotation2d.kZero) {
        subsystemsRun(ALGAE_FRONT_BARGE);
      } else {
        subsystemsRun(ALGAE_BACK_BARGE);
      }
      intake.setWantedState(Intake.WantedState.EJECT_ALGAE);
    }
  }

  private void scoreAlgaeProcessor() {
    Rotation2d rotation = FieldConstants.isBlueAlliance() ? Rotation2d.k180deg : Rotation2d.kZero;
    if (RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation().getDegrees()
        < 0) {
      rotation = Rotation2d.kCW_90deg;
    } else {
      rotation = Rotation2d.kCCW_90deg;
    }
    drive.setTargetRotation(rotation);
    if (drive.isAtDesiredRotation()) {
      subsystemsRun(ALGAE_FRONT_PROCESSOR);
    }
    intake.setWantedState(Intake.WantedState.EJECT_ALGAE);
  }

  public boolean isReadyToEject() {
    return readyToScoreDebouncer.calculate(
        drive.isAtDriveToPointSetpoint()
            && drive.isAtDesiredRotation(Units.degreesToRadians(2.0))
            && drive.isStopped()
            && elevator.atGoal());
  }

  public boolean isReadyToEjectInAutoPeriod() {
    return readyToScoreDebouncerAuto.calculate(
        elevator.atGoal() && drive.isAtEndOfChoreoTrajectoryOrDriveToPoint() && drive.isStopped());
  }

  public boolean isReadyToIntakeCountdown() {
    return intakeDebouncerAuto.calculate(
        elevator.atGoal() && drive.isAtEndOfChoreoTrajectoryOrDriveToPoint() && drive.isStopped());
  }

  public boolean driveToScoringPose(
      ScoringSide scoringSide, ScoringDirection scoringDirection, int scoringLevel) {
    Pose2d desiredPoseToDriveTo =
        !(scoringLevel == 1)
            ? FieldConstants.getDesiredFinalScoringPoseForCoral(
                RobotState.getInstance().getClosestTagId(),
                scoringSide,
                scoringDirection,
                scoringLevel)
            : FieldConstants.getDesiredPointToDriveToForL1Scoring(
                RobotState.getInstance().getClosestTagId(), scoringSide);

    drive.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(desiredPoseToDriveTo, 3.0);
    Logger.recordOutput("Choreographer/DesiredPointToDriveTo", desiredPoseToDriveTo);
    return true;
  }

  public boolean driveToScoringPoseL4(ScoringSide scoringSide, ScoringDirection scoringDirection) {
    Pose2d desiredPoseToDriveTo =
        FieldConstants.getDesiredFinalScoringPoseForCoral(
            RobotState.getInstance().getClosestTagId(), scoringSide, scoringDirection, 4);

    drive.setDesiredPoseForDriveToPointWithConstraints(desiredPoseToDriveTo, 0.5, 3.0);
    Logger.recordOutput("Choreographer/DesiredPointToDriveTo", desiredPoseToDriveTo);
    return true;
  }

  public boolean driveToScoringPose(
      ReefConstants.ReefFaces face, ScoringSide scoringSide, int scoringLevel) {
    var map =
        FieldConstants.isBlueAlliance()
            ? ReefConstants.blueAllianceReefFacesToIds
            : ReefConstants.redAllianceReefFacesToIds;
    var id = map.get(face);
    Pose2d desiredPoseToDriveTo =
        !(scoringLevel == 1)
            ? FieldConstants.getDesiredFinalScoringPoseForCoral(
                id, scoringSide, scoringDirection, scoringLevel)
            : FieldConstants.getDesiredPointToDriveToForL1Scoring(id, scoringSide);

    drive.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(desiredPoseToDriveTo, 3.0);
    Logger.recordOutput("Choreographer/DesiredPointToDriveTo", desiredPoseToDriveTo);
    return true;
  }

  public boolean driveToScoringPose(
      ReefConstants.ReefFaces face,
      ScoringSide scoringSide,
      ScoringDirection scoringDirection,
      int scoringLevel) {
    var map =
        FieldConstants.isBlueAlliance()
            ? ReefConstants.blueAllianceReefFacesToIds
            : ReefConstants.redAllianceReefFacesToIds;
    var id = map.get(face);
    Pose2d desiredPoseToDriveTo =
        !(scoringLevel == 1)
            ? FieldConstants.getDesiredFinalScoringPoseForCoral(
                id, scoringSide, scoringDirection, scoringLevel)
            : FieldConstants.getDesiredPointToDriveToForL1Scoring(id, scoringSide);

    drive.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(desiredPoseToDriveTo, 3.0);
    Logger.recordOutput("Choreographer/DesiredPointToDriveTo", desiredPoseToDriveTo);
    return true;
  }

  public boolean reefTagVisible() {
    int desiredId = RobotState.getInstance().getClosestTagId();
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

      Rotation2d rotation = new Rotation2d();

      if (scoringDirection == ScoringDirection.FRONT) {
        rotation = Rotation2d.k180deg;
      } else {
        rotation = Rotation2d.kZero;
      }

      Pose2d transformedPose =
          tagPose.plus(new Transform2d(offsetFromTag.getX(), offsetFromTag.getY(), rotation));

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

      Rotation2d rotation = new Rotation2d();

      if (scoringDirection == ScoringDirection.FRONT) {
        rotation = Rotation2d.k180deg;
      } else {
        rotation = Rotation2d.kZero;
      }

      Pose2d transformedPose =
          tagPose.plus(new Transform2d(offsetFromTag.getX(), offsetFromTag.getY(), rotation));

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

      Rotation2d rotation = new Rotation2d();

      if (scoringDirection == ScoringDirection.FRONT) {
        rotation = Rotation2d.k180deg;
      } else {
        rotation = Rotation2d.kZero;
      }

      Pose2d transformedPose =
          tagPose.plus(new Transform2d(offsetFromTag.getX(), offsetFromTag.getY(), rotation));

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
    double currentElevator = elevator.getPosition();
    double targetElevator = position.elevatorHeightMeters();
    Rotation2d targetPivot = position.pivotAngle();
    Rotation2d targetWrist = position.differentialWristAngle();
    Rotation2d targetDiffPivot = position.differentialPivotAngle();

    boolean goingUp = currentElevator < targetElevator - 0.01;
    boolean goingDown = currentElevator > targetElevator + 0.01;

    // Always update slapdowns
    coralSlapdown.setWantedState(
        CoralSlapdown.WantedState.MOVE_TO_POSITION, position.coralSlapdownAngle());
    algaeSlapdown.setWantedState(
        AlgaeSlapdown.WantedState.MOVE_TO_POSITION, position.algaeSlapdownAngle());

    if (goingUp) {
      // 1. Move pivot first
      pivot.setWantedState(Pivot.WantedState.MOVE_TO_POSITION, targetPivot);

      // 2. Once pivot is at goal, move elevator
      if (pivot.atGoal()) {
        elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, targetElevator);

        Rotation2d clampedDiffPivot = targetDiffPivot;
        if (!elevator.atGoal()) {
          clampedDiffPivot =
              Rotation2d.fromDegrees(Math.max(-15, Math.min(15, targetDiffPivot.getDegrees())));
        }
        differential.setWantedState(
            DifferentialWristPivot.WantedState.MOVE_TO_POSITIONS,
            clampedDiffPivot,
            new Rotation2d());

        // 3. Once elevator is at goal, move wrist
        if (elevator.atGoal()) {
          differential.setWantedState(
              DifferentialWristPivot.WantedState.MOVE_TO_POSITIONS, targetDiffPivot, targetWrist);
        }
      }
    } else if (goingDown) {
      // 1. Move wrist and elevator together
      elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, targetElevator);
      differential.setWantedState(
          DifferentialWristPivot.WantedState.MOVE_TO_POSITIONS, targetDiffPivot, targetWrist);

      if (elevator.atGoal()) {
        pivot.setWantedState(Pivot.WantedState.MOVE_TO_POSITION, targetPivot);
      }
    } else {
      // Already at target elevator height, just make sure everything else is at goal
      pivot.setWantedState(Pivot.WantedState.MOVE_TO_POSITION, targetPivot);
      differential.setWantedState(
          DifferentialWristPivot.WantedState.MOVE_TO_POSITIONS, targetDiffPivot, targetWrist);
    }
  }

  public void subsystemsRunAlgaeSlapdownLater(Position position) {
    double currentElevator = elevator.getPosition();
    double targetElevator = position.elevatorHeightMeters();
    Rotation2d targetPivot = position.pivotAngle();
    Rotation2d targetWrist = position.differentialWristAngle();
    Rotation2d targetDiffPivot = position.differentialPivotAngle();

    boolean goingUp = currentElevator < targetElevator - 0.01;
    boolean goingDown = currentElevator > targetElevator + 0.01;

    // Always update slapdowns
    coralSlapdown.setWantedState(
        CoralSlapdown.WantedState.MOVE_TO_POSITION, position.coralSlapdownAngle());

    if (goingUp) {
      // 1. Move pivot first
      pivot.setWantedState(Pivot.WantedState.MOVE_TO_POSITION, targetPivot);

      // 2. Once pivot is at goal, move elevator
      if (pivot.atGoal()) {
        elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, targetElevator);

        Rotation2d clampedDiffPivot = targetDiffPivot;
        if (!elevator.atGoal()) {
          clampedDiffPivot =
              Rotation2d.fromDegrees(Math.max(-15, Math.min(15, targetDiffPivot.getDegrees())));
        }
        differential.setWantedState(
            DifferentialWristPivot.WantedState.MOVE_TO_POSITIONS,
            clampedDiffPivot,
            new Rotation2d());

        // 3. Once elevator is at goal, move wrist
        if (elevator.atGoal()) {
          differential.setWantedState(
              DifferentialWristPivot.WantedState.MOVE_TO_POSITIONS, targetDiffPivot, targetWrist);
        }
      }
    } else if (goingDown) {
      // 1. Move wrist and elevator together
      elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, targetElevator);
      differential.setWantedState(
          DifferentialWristPivot.WantedState.MOVE_TO_POSITIONS, targetDiffPivot, targetWrist);

      if (elevator.atGoal()) {
        pivot.setWantedState(Pivot.WantedState.MOVE_TO_POSITION, targetPivot);
      }
    } else {
      // Already at target elevator height, just make sure everything else is at goal
      pivot.setWantedState(Pivot.WantedState.MOVE_TO_POSITION, targetPivot);
      differential.setWantedState(
          DifferentialWristPivot.WantedState.MOVE_TO_POSITIONS, targetDiffPivot, targetWrist);
    }

    if (pivot.atGoal(Rotation2d.fromDegrees(1))) {
      algaeSlapdownRun(position);
    }
  }

  public void subsystemsRunDirect(Position position) {
    pivot.setWantedState(Pivot.WantedState.MOVE_TO_POSITION, position.pivotAngle());
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, position.elevatorHeightMeters());
    coralSlapdown.setWantedState(
        CoralSlapdown.WantedState.MOVE_TO_POSITION, position.coralSlapdownAngle());
    algaeSlapdown.setWantedState(
        AlgaeSlapdown.WantedState.MOVE_TO_POSITION, position.algaeSlapdownAngle());
    differential.setWantedState(
        DifferentialWristPivot.WantedState.MOVE_TO_POSITIONS,
        position.differentialPivotAngle(),
        position.differentialWristAngle());
  }

  public void differentialRun(Position position) {
    differential.setWantedState(
        DifferentialWristPivot.WantedState.MOVE_TO_POSITIONS,
        position.differentialPivotAngle(),
        position.differentialWristAngle());
  }

  public void coralSlapdownRun(Position position) {
    coralSlapdown.setWantedState(
        CoralSlapdown.WantedState.MOVE_TO_POSITION, position.coralSlapdownAngle());
  }

  public void algaeSlapdownRun(Position position) {
    algaeSlapdown.setWantedState(
        AlgaeSlapdown.WantedState.MOVE_TO_POSITION, position.algaeSlapdownAngle());
  }

  public void elevatorRun(Position position) {
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, position.elevatorHeightMeters());
  }

  public void pivotRun(Position position) {
    pivot.setWantedState(Pivot.WantedState.MOVE_TO_POSITION, position.pivotAngle());
  }

  public boolean allAtGoals() {
    return differential.atGoal()
        && elevator.atGoal()
        && pivot.atGoal()
        && coralSlapdown.atGoal()
        && algaeSlapdown.atGoal()
        && drive.isAtDriveToPointSetpoint();
  }

  public boolean mechanismsAtGoals() {
    return differential.atGoal()
        && elevator.atGoal()
        && pivot.atGoal()
        && coralSlapdown.atGoal()
        && algaeSlapdown.atGoal();
  }

  private void handleCoralLocationChoreography() {
    switch (wantedCoralLocation) {
      case SLAPDOWN:
        if (!intake.hasCoralSlapdown() && intake.hasCoralClaw()) {
          subsystemsRun(CORAL_HANDOFF);
          if (mechanismsAtGoals()) {
            intake.setWantedState(Intake.WantedState.RETURN_CORAL);
            if (Robot.isSimulation()) {
              intake.setHasCoralSlapdown(simCoralDebouncer.calculate(true));
              intake.setHasCoralClaw(!simCoralDebouncer.calculate(true));
            }
          }
        }
        break;
      case CLAW:
        if (!intake.hasCoralClaw() && intake.hasCoralSlapdown()) {
          subsystemsRun(CORAL_HANDOFF);
          if (mechanismsAtGoals()) {
            intake.setWantedState(Intake.WantedState.HANDOFF_CORAL);
            if (Robot.isSimulation()) {
              intake.setHasCoralSlapdown(!simCoralDebouncer.calculate(true));
              intake.setHasCoralClaw(simCoralDebouncer.calculate(true));
            }
          }
        }
        break;
    }
  }
}
