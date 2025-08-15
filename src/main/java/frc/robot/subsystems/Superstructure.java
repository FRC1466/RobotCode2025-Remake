// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.SuperstructureConstants.*;
import static frc.robot.constants.SuperstructurePositions.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ReefConstants;
import frc.robot.constants.SuperstructureConstants;
import frc.robot.constants.SuperstructureConstants.ScoringDirection;
import frc.robot.constants.SuperstructureConstants.ScoringSide;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.overridePublisher.OverridePublisher;
import frc.robot.subsystems.slapdown.Slapdown;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.Wrist.ForceDirection;
import frc.robot.util.Position;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Intake intake;
  private final Wrist wrist;
  private final Elevator elevator;
  private final Slapdown slapdown;
  private final OverridePublisher overrides;
  private final Vision vision;

  private static final double defaultTeleopTranslationCoeffecient = 1.0;

  private static final Debouncer simCoralDebouncer =
      new Debouncer(.5, Debouncer.DebounceType.kRising);
  private static final Debouncer simAlgaeDebouncer =
      new Debouncer(.5, Debouncer.DebounceType.kRising);

  private static final Debouncer readyToScoreDebouncer =
      new Debouncer(.5, Debouncer.DebounceType.kRising);

  private static final Debouncer readyToScoreDebouncerAuto =
      new Debouncer(.5, Debouncer.DebounceType.kRising);

  private static final Debouncer intakeDebouncerAuto =
      new Debouncer(.1, Debouncer.DebounceType.kRising);

  private final Timer coralL1TopTimer = new Timer();

  public enum WantedCoralLocation {
    CLAW,
    SLAPDOWN
  }

  public enum CurrentCoralLocation {
    CLAW,
    SLAPDOWN,
    NONE
  }

  public enum WantedSuperState {
    HOME,
    STOPPED,
    DEFAULT_STATE,
    INTAKE_CORAL_FROM_STATION,
    INTAKE_CORAL_FROM_GROUND,
    SCORE_LEFT_L1,
    SCORE_LEFT_L2,
    SCORE_LEFT_L3,
    SCORE_LEFT_L4,
    SCORE_RIGHT_L1,
    SCORE_RIGHT_L2,
    SCORE_RIGHT_L3,
    SCORE_RIGHT_L4,
    INTAKE_ALGAE_REEF,
    INTAKE_ALGAE_GROUND,
    INTAKE_ALGAE_ICE_CREAM,
    MOVE_ALGAE_TO_NET_POSITION,
    SCORE_ALGAE_IN_NET,
    MOVE_ALGAE_TO_PROCESSOR_POSITION,
    SCORE_ALGAE_IN_PROCESSOR,
    EJECT_ALGAE,
    EJECT_CORAL
  }

  public enum CurrentSuperState {
    HOME,
    STOPPED,
    NO_PIECE_TELEOP,
    HOLDING_CORAL_TELEOP,
    NO_PIECE_AUTO,
    HOLDING_CORAL_AUTO,
    HOLDING_ALGAE,
    INTAKE_CORAL_FROM_STATION,
    INTAKE_CORAL_FROM_GROUND,
    SCORE_LEFT_TELEOP_L1,
    SCORE_LEFT_TELEOP_L2,
    SCORE_LEFT_TELEOP_L3,
    SCORE_LEFT_TELEOP_L4,
    SCORE_RIGHT_TELEOP_L1,
    SCORE_RIGHT_TELEOP_L2,
    SCORE_RIGHT_TELEOP_L3,
    SCORE_RIGHT_TELEOP_L4,
    SCORE_LEFT_AUTO_L1,
    SCORE_LEFT_AUTO_L2,
    SCORE_LEFT_AUTO_L3,
    SCORE_LEFT_AUTO_L4,
    SCORE_RIGHT_AUTO_L1,
    SCORE_RIGHT_AUTO_L2,
    SCORE_RIGHT_AUTO_L3,
    SCORE_RIGHT_AUTO_L4,
    INTAKE_ALGAE_REEF,
    INTAKE_ALGAE_GROUND,
    INTAKE_ALGAE_ICE_CREAM,
    MOVE_ALGAE_TO_NET_POSITION,
    SCORE_ALGAE_IN_NET,
    MOVE_ALGAE_TO_PROCESSOR_POSITION,
    SCORE_ALGAE_IN_PROCESSOR,
    EJECT_ALGAE,
    EJECT_CORAL
  }

  private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
  private CurrentSuperState previousSuperState;

  @Setter private WantedCoralLocation wantedCoralLocation = WantedCoralLocation.CLAW;

  private ScoringDirection scoringDirection = ScoringDirection.LEFT;

  private boolean hasDriveReachedIntermediatePoseForReefAlgaePickup = false;

  private boolean coralEject = false;

  public Superstructure(
      Drive drive,
      Intake intake,
      Elevator elevator,
      Wrist wrist,
      Slapdown slapdown,
      OverridePublisher overrides,
      Vision vision) {
    this.drive = drive;
    this.intake = intake;
    this.wrist = wrist;
    this.elevator = elevator;
    this.slapdown = slapdown;
    this.overrides = overrides;
    this.vision = vision;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);
    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
    Logger.recordOutput("Superstructure/PreviousSuperState", previousSuperState);

    Logger.recordOutput("Superstructure/WantedCoralLocation", wantedCoralLocation);

    currentSuperState = handStateTransitions();
    applyStates();
  }

  private CurrentSuperState handStateTransitions() {
    previousSuperState = currentSuperState;
    switch (wantedSuperState) {
      default:
        currentSuperState = CurrentSuperState.STOPPED;
        break;
      case HOME:
        currentSuperState = CurrentSuperState.HOME;
        break;
      case INTAKE_CORAL_FROM_STATION:
        currentSuperState = CurrentSuperState.INTAKE_CORAL_FROM_STATION;
        break;
      case INTAKE_CORAL_FROM_GROUND:
        currentSuperState = CurrentSuperState.INTAKE_CORAL_FROM_GROUND;
        break;
      case DEFAULT_STATE:
        if (intake.hasCoralClaw() || intake.hasCoralSlapdown()) {
          if (DriverStation.isAutonomous()) {
            currentSuperState = CurrentSuperState.HOLDING_CORAL_AUTO;
          } else {
            currentSuperState = CurrentSuperState.HOLDING_CORAL_TELEOP;
          }
        } else if (intake.hasAlgae()) {
          currentSuperState = CurrentSuperState.HOLDING_ALGAE;
        } else {
          if (DriverStation.isAutonomous()) {
            currentSuperState = CurrentSuperState.NO_PIECE_AUTO;
          } else {
            currentSuperState = CurrentSuperState.NO_PIECE_TELEOP;
          }
        }
        break;
      case SCORE_LEFT_L1:
        currentSuperState =
            DriverStation.isAutonomous()
                ? CurrentSuperState.SCORE_LEFT_AUTO_L1
                : CurrentSuperState.SCORE_LEFT_TELEOP_L1;
        break;
      case SCORE_LEFT_L2:
        currentSuperState =
            DriverStation.isAutonomous()
                ? CurrentSuperState.SCORE_LEFT_AUTO_L2
                : CurrentSuperState.SCORE_LEFT_TELEOP_L2;
        break;
      case SCORE_LEFT_L3:
        currentSuperState =
            DriverStation.isAutonomous()
                ? CurrentSuperState.SCORE_LEFT_AUTO_L3
                : CurrentSuperState.SCORE_LEFT_TELEOP_L3;
        break;
      case SCORE_LEFT_L4:
        currentSuperState =
            DriverStation.isAutonomous()
                ? CurrentSuperState.SCORE_LEFT_AUTO_L4
                : CurrentSuperState.SCORE_LEFT_TELEOP_L4;
        break;
      case SCORE_RIGHT_L1:
        currentSuperState =
            DriverStation.isAutonomous()
                ? CurrentSuperState.SCORE_RIGHT_AUTO_L1
                : CurrentSuperState.SCORE_RIGHT_TELEOP_L1;
        break;
      case SCORE_RIGHT_L2:
        currentSuperState =
            DriverStation.isAutonomous()
                ? CurrentSuperState.SCORE_RIGHT_AUTO_L2
                : CurrentSuperState.SCORE_RIGHT_TELEOP_L2;
        break;
      case SCORE_RIGHT_L3:
        currentSuperState =
            DriverStation.isAutonomous()
                ? CurrentSuperState.SCORE_RIGHT_AUTO_L3
                : CurrentSuperState.SCORE_RIGHT_TELEOP_L3;
        break;
      case SCORE_RIGHT_L4:
        currentSuperState =
            DriverStation.isAutonomous()
                ? CurrentSuperState.SCORE_RIGHT_AUTO_L4
                : CurrentSuperState.SCORE_RIGHT_TELEOP_L4;
        break;
      case INTAKE_ALGAE_ICE_CREAM:
        currentSuperState = CurrentSuperState.INTAKE_ALGAE_ICE_CREAM;
        break;
      case INTAKE_ALGAE_REEF:
        currentSuperState = CurrentSuperState.INTAKE_ALGAE_REEF;
        break;
      case INTAKE_ALGAE_GROUND:
        currentSuperState = CurrentSuperState.INTAKE_ALGAE_GROUND;
        break;
      case MOVE_ALGAE_TO_NET_POSITION:
        currentSuperState = CurrentSuperState.MOVE_ALGAE_TO_NET_POSITION;
        break;
      case MOVE_ALGAE_TO_PROCESSOR_POSITION:
        currentSuperState = CurrentSuperState.MOVE_ALGAE_TO_PROCESSOR_POSITION;
        break;
      case SCORE_ALGAE_IN_NET:
        currentSuperState = CurrentSuperState.SCORE_ALGAE_IN_NET;
        break;
      case SCORE_ALGAE_IN_PROCESSOR:
        currentSuperState = CurrentSuperState.SCORE_ALGAE_IN_PROCESSOR;
        break;
    }
    return currentSuperState;
  }

  private void applyStates() {
    scoringDirection = RobotState.getInstance().getFacingSideRelativeToClosestTag();
    Logger.recordOutput("Superstructure/ScoringDirection", scoringDirection);
    if (previousSuperState != currentSuperState) {
      wrist.clearForceDirection();
      simAlgaeDebouncer.calculate(false);
      simCoralDebouncer.calculate(false);
      readyToScoreDebouncer.calculate(false);
      readyToScoreDebouncerAuto.calculate(false);
      intakeDebouncerAuto.calculate(false);
      hasDriveReachedIntermediatePoseForReefAlgaePickup = false;
    }

    switch (currentSuperState) {
      case HOME:
        home();
        break;
      case INTAKE_CORAL_FROM_STATION:
        // intakeCoralFromStation();
        break;
      case INTAKE_CORAL_FROM_GROUND:
        intakeCoralFromGround();
        break;
      case NO_PIECE_TELEOP:
        noPiece();
        break;
      case NO_PIECE_AUTO:
        noPieceAuto();
        break;
      case HOLDING_CORAL_AUTO:
        holdingCoralAuto();
        break;
      case HOLDING_CORAL_TELEOP:
        holdingCoral();
        break;
      case HOLDING_ALGAE:
        holdingAlgae();
        break;
      case SCORE_LEFT_TELEOP_L1:
        scoreL1Teleop(ScoringSide.LEFT);
        break;
      case SCORE_LEFT_TELEOP_L2:
        scoreL2Teleop(ScoringSide.LEFT);
        break;
      case SCORE_LEFT_TELEOP_L3:
        scoreL3Teleop(ScoringSide.LEFT);
        break;
      case SCORE_LEFT_TELEOP_L4:
        scoreL4Teleop(ScoringSide.LEFT);
        break;
      case SCORE_RIGHT_TELEOP_L1:
        scoreL1Teleop(ScoringSide.RIGHT);
        break;
      case SCORE_RIGHT_TELEOP_L2:
        scoreL2Teleop(ScoringSide.RIGHT);
        break;
      case SCORE_RIGHT_TELEOP_L3:
        scoreL3Teleop(ScoringSide.RIGHT);
        break;
      case SCORE_RIGHT_TELEOP_L4:
        scoreL4Teleop(ScoringSide.RIGHT);
        break;
      case SCORE_LEFT_AUTO_L1:
        scoreL1Auto(ScoringSide.LEFT);
        break;
      case SCORE_LEFT_AUTO_L2:
        scoreL2Auto(ScoringSide.LEFT);
        break;
      case SCORE_LEFT_AUTO_L3:
        scoreL3Auto(ScoringSide.LEFT);
        break;
      case SCORE_LEFT_AUTO_L4:
        scoreL4Auto(ScoringSide.LEFT);
        break;
      case SCORE_RIGHT_AUTO_L1:
        scoreL1Auto(ScoringSide.RIGHT);
        break;
      case SCORE_RIGHT_AUTO_L2:
        scoreL2Auto(ScoringSide.RIGHT);
        break;
      case SCORE_RIGHT_AUTO_L3:
        scoreL3Auto(ScoringSide.RIGHT);
        break;
      case SCORE_RIGHT_AUTO_L4:
        scoreL4Auto(ScoringSide.RIGHT);
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

  private void home() {}

  private void stopped() {
    wrist.setWantedState(Wrist.WantedState.IDLE);
    elevator.setWantedState(Elevator.WantedState.IDLE);
    intake.setWantedState(Intake.WantedState.OFF);
  }

  private void holdingAlgae() {
    subsystemsRun(ALGAE_STOW);

    drive.setWantedState(Drive.WantedState.TELEOP_DRIVE);
    drive.setTeleopVelocityCoefficient(defaultTeleopTranslationCoeffecient);
    drive.setRotationVelocityCoefficient(1.0);

    intake.setWantedState(
        wrist.atGoal() && elevator.atGoal()
            ? Intake.WantedState.HOLD_ALGAE
            : Intake.WantedState.HOLD_ALGAE_HARDER);
  }

  private void holdingCoral() {
    coralL1TopTimer.stop();
    coralEject = false;
    subsystemsRun(CORAL_STOW);
    wristRunExact(CORAL_STOW);
    intake.setWantedState(Intake.WantedState.HOLD_CORAL);
    drive.setWantedState(Drive.WantedState.TELEOP_DRIVE);
    drive.setTeleopVelocityCoefficient(defaultTeleopTranslationCoeffecient);
    drive.setRotationVelocityCoefficient(1.0);
    handleCoralLocationTransitions();
  }

  private void holdingCoralAuto() {
    coralEject = false;
    subsystemsRun(CORAL_STOW);
    wristRunExact(CORAL_STOW);
    intake.setWantedState(Intake.WantedState.HOLD_CORAL);
    handleCoralLocationTransitions();
  }

  private void noPiece() {
    wrist.clearForceDirection();
    coralL1TopTimer.stop();
    subsystemsRun(CORAL_STOW);
    wristRunExact(CORAL_STOW);
    intake.setWantedState(Intake.WantedState.OFF);
    drive.setWantedState(Drive.WantedState.TELEOP_DRIVE);
    drive.setTeleopVelocityCoefficient(defaultTeleopTranslationCoeffecient);
    drive.setRotationVelocityCoefficient(1.0);
  }

  private void noPieceAuto() {
    wrist.clearForceDirection();
    subsystemsRun(CORAL_STOW);
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
            ? scoringDirection == ScoringDirection.LEFT
                ? ALGAE_LEFT_INTAKE_L2
                : ALGAE_RIGHT_INTAKE_L2
            : scoringDirection == ScoringDirection.LEFT
                ? ALGAE_LEFT_INTAKE_L3
                : ALGAE_RIGHT_INTAKE_L3);

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

  private void intakeAlgaeFromGround() {}

  private void intakeAlgaeIceCream() {}

  private void scoreL1Teleop(ScoringSide scoringSide) {
    setWantedCoralLocation(WantedCoralLocation.SLAPDOWN);
    if (intake.hasCoralSlapdown()) {
      if (!overrides.isReefOverride()) {
        driveToScoringPose(scoringSide, scoringDirection, true);
      }
      subsystemsRun(L1);
      if (isReadyToEject()) {
        coralEject = true;
      }
      if (coralEject) {
        intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
      }
    } else {
      handleCoralLocationTransitions();
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
        driveToScoringPose(scoringSide, scoringDirection, false);
      }
      if (scoringDirection == ScoringDirection.RIGHT) {
        wrist.setForceDirection(ForceDirection.COUNTERCLOCKWISE);
        subsystemsRun(L2_LEFT_HOLD);
      } else {
        wrist.setForceDirection(ForceDirection.CLOCKWISE);
        subsystemsRun(L2_RIGHT_HOLD);
      }

      if (isReadyToEject()) {
        coralEject = true;
      }
      if (coralEject) {
        if (overrides.isReefOverride() == false) {
          if (scoringDirection == ScoringDirection.RIGHT) {
            subsystemsRun(L2_LEFT_SCORE);
          } else {
            subsystemsRun(L2_RIGHT_SCORE);
          }
          intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
        }
      }
    } else {
      handleCoralLocationTransitions();
    }
  }

  private void scoreL2Auto(ScoringSide scoringSide) {
    if (scoringDirection == ScoringDirection.LEFT) {
      subsystemsRun(L2_LEFT_HOLD);
    } else {
      subsystemsRun(L2_RIGHT_HOLD);
    }

    if (isReadyToEjectInAutoPeriod()) {
      coralEject = true;
    }

    if (coralEject) {
      if (scoringDirection == ScoringDirection.LEFT) {
        subsystemsRun(L2_LEFT_SCORE);
      } else {
        subsystemsRun(L2_RIGHT_SCORE);
      }
      intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
    }
  }

  private void scoreL3Teleop(ScoringSide scoringSide) {
    setWantedCoralLocation(WantedCoralLocation.CLAW);
    if (intake.hasCoralClaw()) {
      if (!overrides.isReefOverride()) {
        driveToScoringPose(scoringSide, scoringDirection, false);
      }
      if (scoringDirection == ScoringDirection.RIGHT) {
        wrist.setForceDirection(ForceDirection.COUNTERCLOCKWISE);
        subsystemsRun(L3_LEFT_HOLD);
      } else {
        wrist.setForceDirection(ForceDirection.CLOCKWISE);
        subsystemsRun(L3_RIGHT_HOLD);
      }
      if (isReadyToEject()) {
        coralEject = true;
      }
      if (coralEject) {
        if (overrides.isReefOverride() == false) {
          if (scoringDirection == ScoringDirection.RIGHT) {
            subsystemsRun(L3_LEFT_SCORE);
          } else {
            subsystemsRun(L3_RIGHT_SCORE);
          }
          handleCoralLocationTransitions();
        }
      }
    } else {
      currentSuperState = CurrentSuperState.HOLDING_CORAL_TELEOP;
    }
  }

  private void scoreL3Auto(ScoringSide scoringSide) {
    if (scoringDirection == ScoringDirection.LEFT) {
      subsystemsRun(L3_LEFT_HOLD);
    } else {
      subsystemsRun(L3_RIGHT_HOLD);
    }

    if (isReadyToEjectInAutoPeriod()) {
      coralEject = true;
    }

    if (coralEject) {
      if (scoringDirection == ScoringDirection.LEFT) {
        subsystemsRun(L3_LEFT_SCORE);
      } else {
        subsystemsRun(L3_RIGHT_SCORE);
      }
      intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
    }
  }

  private void scoreL4Teleop(ScoringSide scoringSide) {
    setWantedCoralLocation(WantedCoralLocation.CLAW);
    if (intake.hasCoralClaw()) {
      if (!overrides.isReefOverride()) {
        driveToScoringPose(scoringSide, scoringDirection, false);
      }
      if (scoringDirection == ScoringDirection.RIGHT) {
        wrist.setForceDirection(ForceDirection.COUNTERCLOCKWISE);
        subsystemsRun(L4_LEFT_HOLD);
      } else {
        wrist.setForceDirection(ForceDirection.CLOCKWISE);
        subsystemsRun(L4_RIGHT_HOLD);
      }
      if (isReadyToEject()) {
        coralEject = true;
      }
      if (coralEject) {
        if (overrides.isReefOverride() == false) {
          if (scoringDirection == ScoringDirection.RIGHT) {
            subsystemsRun(L4_LEFT_SCORE);
          } else {
            subsystemsRun(L4_RIGHT_SCORE);
          }
          intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
        }
      }
    } else {
      handleCoralLocationTransitions();
    }
  }

  private void scoreL4Auto(ScoringSide scoringSide) {
    if (scoringDirection == ScoringDirection.LEFT) {
      subsystemsRun(L3_LEFT_HOLD);
    } else {
      subsystemsRun(L3_RIGHT_HOLD);
    }

    if (isReadyToEjectInAutoPeriod()) {
      coralEject = true;
    }

    if (coralEject) {
      if (scoringDirection == ScoringDirection.LEFT) {
        subsystemsRun(L3_LEFT_SCORE);
      } else {
        subsystemsRun(L3_RIGHT_SCORE);
      }
      intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
    }
  }

  private void moveAlgaeToNetPosition() {
    Rotation2d rotation =
        FieldConstants.isBlueAlliance() ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg;
    if (RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation().getDegrees()
        > 0) {
      rotation = Rotation2d.kCCW_90deg;
    } else {
      rotation = Rotation2d.kCW_90deg;
    }
    drive.setTargetRotation(rotation);

    intake.setWantedState(Intake.WantedState.HOLD_ALGAE);
    drive.setTeleopVelocityCoefficient(0.4);

    if (drive.isAtDesiredRotation()) {
      if (rotation == Rotation2d.kCW_90deg) {
        subsystemsRun(ALGAE_LEFT_BARGE);
      } else {
        subsystemsRun(ALGAE_RIGHT_BARGE);
      }
    }
  }

  private void moveAlgaeToProcessorPosition() {
    Rotation2d rotation = FieldConstants.isBlueAlliance() ? Rotation2d.k180deg : Rotation2d.kZero;
    if (Math.abs(
            RobotState.getInstance()
                .getRobotPoseFromSwerveDriveOdometry()
                .getRotation()
                .getDegrees())
        > 90) {
      rotation = Rotation2d.k180deg;
    } else {
      rotation = Rotation2d.kZero;
    }
    drive.setTargetRotation(rotation);
    if (drive.isAtDesiredRotation()) {
      if (FieldConstants.isOnBlueAlliance(
          RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry())) {
        if (rotation == Rotation2d.kZero) {
          subsystemsRun(ALGAE_LEFT_PROCESSOR);
        } else {
          subsystemsRun(ALGAE_RIGHT_PROCESSOR);
        }
      } else {
        if (rotation == Rotation2d.k180deg) {
          subsystemsRun(ALGAE_LEFT_PROCESSOR);
        } else {
          subsystemsRun(ALGAE_RIGHT_PROCESSOR);
        }
      }
    }
  }

  private void scoreAlgaeNet() {
    drive.setTeleopVelocityCoefficient(0.0);
    Rotation2d rotation =
        FieldConstants.isBlueAlliance() ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg;
    if (RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation().getDegrees()
        > 0) {
      rotation = Rotation2d.kCCW_90deg;
    } else {
      rotation = Rotation2d.kCW_90deg;
    }
    if (drive.isAtDesiredRotation()) {
      if (rotation == Rotation2d.kCW_90deg) {
        subsystemsRun(ALGAE_LEFT_BARGE);
      } else {
        subsystemsRun(ALGAE_RIGHT_BARGE);
      }
      intake.setWantedState(Intake.WantedState.EJECT_ALGAE);
    }
  }

  private void scoreAlgaeProcessor() {
    Rotation2d rotation = FieldConstants.isBlueAlliance() ? Rotation2d.k180deg : Rotation2d.kZero;
    if (Math.abs(
            RobotState.getInstance()
                .getRobotPoseFromSwerveDriveOdometry()
                .getRotation()
                .getDegrees())
        > 90) {
      rotation = Rotation2d.k180deg;
    } else {
      rotation = Rotation2d.kZero;
    }
    drive.setTargetRotation(rotation);
    if (drive.isAtDesiredRotation()) {
      if (FieldConstants.isOnBlueAlliance(
          RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry())) {
        if (rotation == Rotation2d.kZero) {
          subsystemsRun(ALGAE_LEFT_PROCESSOR);
        } else {
          subsystemsRun(ALGAE_RIGHT_PROCESSOR);
        }
      } else {
        if (rotation == Rotation2d.k180deg) {
          subsystemsRun(ALGAE_LEFT_PROCESSOR);
        } else {
          subsystemsRun(ALGAE_RIGHT_PROCESSOR);
        }
      }
    }
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

  public boolean driveToScoringPose(
      ScoringSide scoringSide, ScoringDirection scoringDirection, boolean isL1) {
    Pose2d desiredPoseToDriveTo =
        !isL1
            ? FieldConstants.getDesiredFinalScoringPoseForCoral(
                RobotState.getInstance().getClosestTagId(), scoringSide, scoringDirection)
            : FieldConstants.getDesiredPointToDriveToForL1Scoring(
                RobotState.getInstance().getClosestTagId(), scoringSide);

    drive.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(desiredPoseToDriveTo, 3.0);
    Logger.recordOutput("Superstructure/DesiredPointToDriveTo", desiredPoseToDriveTo);
    return true;
  }

  public boolean driveToScoringPoseL4(ScoringSide scoringSide, ScoringDirection scoringDirection) {
    Pose2d desiredPoseToDriveTo =
        FieldConstants.getDesiredFinalScoringPoseForCoral(
            RobotState.getInstance().getClosestTagId(), scoringSide, scoringDirection);

    drive.setDesiredPoseForDriveToPointWithConstraints(desiredPoseToDriveTo, 0.5, 3.0);
    Logger.recordOutput("Superstructure/DesiredPointToDriveTo", desiredPoseToDriveTo);
    return true;
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
            ? FieldConstants.getDesiredFinalScoringPoseForCoral(id, scoringSide, scoringDirection)
            : FieldConstants.getDesiredPointToDriveToForL1Scoring(id, scoringSide);

    drive.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(desiredPoseToDriveTo, 3.0);
    Logger.recordOutput("Superstructure/DesiredPointToDriveTo", desiredPoseToDriveTo);
    return true;
  }

  public boolean driveToScoringPose(
      ReefConstants.ReefFaces face,
      ScoringSide scoringSide,
      ScoringDirection scoringDirection,
      boolean isL1) {
    var map =
        FieldConstants.isBlueAlliance()
            ? ReefConstants.blueAllianceReefFacesToIds
            : ReefConstants.redAllianceReefFacesToIds;
    var id = map.get(face);
    Pose2d desiredPoseToDriveTo =
        !isL1
            ? FieldConstants.getDesiredFinalScoringPoseForCoral(id, scoringSide, scoringDirection)
            : FieldConstants.getDesiredPointToDriveToForL1Scoring(id, scoringSide);

    drive.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(desiredPoseToDriveTo, 3.0);
    Logger.recordOutput("Superstructure/DesiredPointToDriveTo", desiredPoseToDriveTo);
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
      double yOffset = 0.0;

      if (scoringDirection == SuperstructureConstants.ScoringDirection.LEFT) {
        yOffset += Units.inchesToMeters(SuperstructureConstants.yOffsetFromPoleForLeft);
      } else {
        yOffset -= Units.inchesToMeters(SuperstructureConstants.yOffsetFromPoleForLeft);
      }

      Translation2d offsetFromTag = new Translation2d(xOffset, yOffset);

      Rotation2d rotation = new Rotation2d();

      if (scoringDirection == ScoringDirection.LEFT) {
        rotation = Rotation2d.kCW_90deg;
      } else {
        rotation = Rotation2d.kCCW_90deg;
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

      double yOffset = 0.0;

      if (scoringDirection == SuperstructureConstants.ScoringDirection.LEFT) {
        yOffset += Units.inchesToMeters(SuperstructureConstants.yOffsetFromPoleForLeft);
      } else {
        yOffset -= Units.inchesToMeters(SuperstructureConstants.yOffsetFromPoleForLeft);
      }

      Translation2d offsetFromTag = new Translation2d(xOffset, yOffset);

      Rotation2d rotation = new Rotation2d();

      if (scoringDirection == ScoringDirection.LEFT) {
        rotation = Rotation2d.kCW_90deg;
      } else {
        rotation = Rotation2d.kCCW_90deg;
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

      double yOffset = 0.0;

      if (scoringDirection == SuperstructureConstants.ScoringDirection.LEFT) {
        yOffset += Units.inchesToMeters(SuperstructureConstants.yOffsetFromPoleForLeft);
      } else {
        yOffset -= Units.inchesToMeters(SuperstructureConstants.yOffsetFromPoleForLeft);
      }

      Translation2d offsetFromTag = new Translation2d(xOffset, yOffset);

      Rotation2d rotation = new Rotation2d();

      if (scoringDirection == ScoringDirection.LEFT) {
        rotation = Rotation2d.kCW_90deg;
      } else {
        rotation = Rotation2d.kCCW_90deg;
      }

      Pose2d transformedPose =
          tagPose.plus(new Transform2d(offsetFromTag.getX(), offsetFromTag.getY(), rotation));

      return transformedPose;
    } else {
      return Pose2d.kZero;
    }
  }

  public void setWantedSuperState(WantedSuperState superState) {
    this.wantedSuperState = superState;
  }

  public Command setStateCommand(WantedSuperState superState) {
    Command commandToReturn = new InstantCommand(() -> setWantedSuperState(superState));
    return commandToReturn;
  }

  public void subsystemsRun(Position position) {
    wrist.setExactAngle(false);
    wrist.setWantedState(Wrist.WantedState.MOVE_TO_POSITION, position.wristAngle());
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, position.elevatorHeightMeters());
    slapdown.setWantedState(Slapdown.WantedState.MOVE_TO_POSITION, position.slapdownAngle());
  }

  public void slapdownRun(Position position) {
    slapdown.setWantedState(Slapdown.WantedState.MOVE_TO_POSITION, position.slapdownAngle());
  }

  public void elevatorRun(Position position) {
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, position.elevatorHeightMeters());
  }

  public void wristRun(Position position) {
    wrist.setExactAngle(false);
    wrist.setWantedState(Wrist.WantedState.MOVE_TO_POSITION, position.wristAngle());
  }

  public void wristRunExact(Position position) {
    wrist.setExactAngle(true);
    wrist.setWantedState(Wrist.WantedState.MOVE_TO_POSITION, position.wristAngle());
  }

  public boolean allAtGoals() {
    return wrist.atGoal()
        && elevator.atGoal()
        && slapdown.atGoal()
        && drive.isAtDriveToPointSetpoint();
  }

  public boolean mechanismsAtGoals() {
    return wrist.atGoal() && elevator.atGoal() && slapdown.atGoal();
  }

  private void handleCoralLocationTransitions() {
    switch (wantedCoralLocation) {
      case SLAPDOWN:
        if (!intake.hasCoralSlapdown()) {
          wristRunExact(CORAL_HANDOFF);
          if (allAtGoals()) {
            intake.setWantedState(Intake.WantedState.RETURN_CORAL);
            if (Robot.isSimulation()) {
              intake.setHasCoralSlapdown(simCoralDebouncer.calculate(true));
              intake.setHasCoralClaw(!simCoralDebouncer.calculate(true));
            }
          }
        }
        break;
      case CLAW:
        if (!intake.hasCoralClaw()) {
          wristRunExact(CORAL_HANDOFF);
          if (allAtGoals()) {
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
