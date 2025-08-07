// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.SuperstructureConstants.*;
import static frc.robot.constants.WristElevatorPoses.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ReefConstants;
import frc.robot.constants.SuperstructureConstants.ScoringSide;
import frc.robot.constants.WristElevatorPoses;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.WantedState;
import frc.robot.subsystems.overridePublisher.OverridePublisher;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.wrist.Wrist;
import java.util.function.BooleanSupplier;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Intake intake;
  private final Wrist wrist;
  private final Elevator elevator;
  private final OverridePublisher overrides;
  private final Vision vision;

  private static final double defaultTeleopTranslationCoeffecient = 1.0;

  private static final Debouncer homeDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
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

  public enum WantedSuperState {
    HOME,
    STOPPED,
    DEFAULT_STATE,
    INTAKE_CORAL_FROM_STATION,
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

  private boolean hasDriveToPointSetPointBeenSet = false;

  private boolean hasDriveReachedIntermediatePoseForReefAlgaePickup = false;

  @Setter private boolean hasDriveReachedIntermediatePoseForCoralScore = false;

  private boolean coralEject = false;

  private BooleanSupplier wristPastSafe = () -> false;

  public Superstructure(
      Drive drive,
      Intake intake,
      Elevator elevator,
      Wrist wrist,
      OverridePublisher overrides,
      Vision vision) {
    this.drive = drive;
    this.intake = intake;
    this.wrist = wrist;
    this.elevator = elevator;
    this.overrides = overrides;
    this.vision = vision;

    this.wristPastSafe = () -> wrist.getAngle().getRadians() > .6;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Superstructure/hasProfileBeenSet", hasDriveToPointSetPointBeenSet);
    Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);
    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
    Logger.recordOutput("Superstructure/PreviousSuperState", previousSuperState);

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
      case DEFAULT_STATE:
        if (intake.hasCoral()) {
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
    if (previousSuperState != currentSuperState) {
      simAlgaeDebouncer.calculate(false);
      simCoralDebouncer.calculate(false);
      readyToScoreDebouncer.calculate(false);
      readyToScoreDebouncerAuto.calculate(false);
      intakeDebouncerAuto.calculate(false);
      hasDriveReachedIntermediatePoseForCoralScore = false;
      hasDriveReachedIntermediatePoseForReefAlgaePickup = false;
    }

    switch (currentSuperState) {
      case HOME:
        home();
        break;
      case INTAKE_CORAL_FROM_STATION:
        intakeCoralFromStation();
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
    hasDriveToPointSetPointBeenSet = false;
    elevatorWristRun(ALGAE_HOLD);

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
    hasDriveToPointSetPointBeenSet = false;
    coralEject = false;
    elevatorWristRun(TRAVEL);
    intake.setWantedState(Intake.WantedState.OFF);
    drive.setWantedState(Drive.WantedState.TELEOP_DRIVE);
    drive.setTeleopVelocityCoefficient(defaultTeleopTranslationCoeffecient);
    drive.setRotationVelocityCoefficient(1.0);
  }

  private void holdingCoralAuto() {
    coralEject = false;
    elevatorWristRun(TRAVEL);
    intake.setWantedState(Intake.WantedState.OFF);
  }

  private void noPiece() {
    coralL1TopTimer.stop();
    hasDriveToPointSetPointBeenSet = false;
    if (elevator.getPosition() > 0.1) {
      elevatorWristRun(TRAVEL);
    } else {
      elevatorWristRun(STOW);
    }
    intake.setWantedState(Intake.WantedState.OFF);
    drive.setWantedState(Drive.WantedState.TELEOP_DRIVE);
    drive.setTeleopVelocityCoefficient(defaultTeleopTranslationCoeffecient);
    drive.setRotationVelocityCoefficient(1.0);
  }

  private void noPieceAuto() {
    elevatorWristRun(TRAVEL);
    intake.setWantedState(Intake.WantedState.OFF);
  }

  private void intakeCoralFromStation() {
    coralEject = false;
    if (DriverStation.isAutonomous()) {
      if (elevator.getPosition() > 0.15) {
        elevatorWristRun(TRAVEL);
      } else {
        elevatorWristRun(STOW);
      }
      if (intake.hasCoral()) {
        elevatorWristRun(TRAVEL);
      }
      intake.setWantedState(Intake.WantedState.INTAKE_CORAL);
    } else {
      if (intake.hasCoral()) {
        drive.setState(Drive.WantedState.TELEOP_DRIVE);
      } else if (RobotState.getInstance()
              .getRobotPoseFromSwerveDriveOdometry()
              .getRotation()
              .getDegrees()
          >= 0) {
        var angleToSnapTo = FieldConstants.isBlueAlliance() ? 54.0 : 126.0;
        drive.setTargetRotation(Rotation2d.fromDegrees(angleToSnapTo));
      } else {
        var angleToSnapTo = FieldConstants.isBlueAlliance() ? -54.0 : -126.0;
        drive.setTargetRotation(Rotation2d.fromDegrees(angleToSnapTo));
      }
      if (elevator.getPosition() > 0.15) {
        elevatorWristRun(TRAVEL);
      } else {
        elevatorWristRun(STOW);
      }
      intake.setWantedState(Intake.WantedState.INTAKE_CORAL);
    }
    if (Robot.isSimulation()) {
      var currentPose = RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry();
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

    var location = levelMap.get(RobotState.getInstance().getClosestRotationToFaceNearestReefFace());
    var level = location.front;

    wrist.setWantedState(Wrist.WantedState.MOVE_TO_POSITION, WristElevatorPoses.TRAVEL.wristAngle);
    if (wristPastSafe.getAsBoolean()) {
      elevatorWristRun(level == ReefConstants.AlgaeIntakeLocation.L2 ? L2_ALGAE : L3_ALGAE);
    }

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
        elevatorWristRun(ALGAE_HOLD);
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
    if (!overrides.isReefOverride()) {
      driveToScoringPose(scoringSide, true);
    }
    wristRun(TRAVEL);

    if (wristPastSafe.getAsBoolean()) {
      elevatorRun(L1);
      if (elevator.atGoal()) {
        elevatorWristRun(L1);
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

  private void scoreL1Auto(ScoringSide scoringSide) {
    wristRun(TRAVEL);
    if (wristPastSafe.getAsBoolean()) {
      elevatorRun(L1);
      if (elevator.atGoal()) {
        elevatorWristRun(L1);
      }

      if (isReadyToEjectInAutoPeriod()) {
        coralEject = true;
      }

      if (coralEject) {
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
      elevatorWristRun(L2);
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

  private void scoreL2Auto(ScoringSide scoringSide) {
    wristRun(TRAVEL);
    if (wristPastSafe.getAsBoolean()) {
      elevatorWristRun(L2);

      if (isReadyToEjectInAutoPeriod()) {
        coralEject = true;
      }

      if (coralEject) {
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
      elevatorWristRun(L3);
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

  private void scoreL3Auto(ScoringSide scoringSide) {
    wristRun(TRAVEL);
    if (wristPastSafe.getAsBoolean()) {
      elevatorWristRun(L3);

      if (isReadyToEjectInAutoPeriod()) {
        coralEject = true;
      }

      if (coralEject) {
        intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
      }
    }
  }

  private void scoreL4Teleop(ScoringSide scoringSide) {
    if (!overrides.isReefOverride()) {
      driveToScoringPoseL4(scoringSide);
    }
    wristRun(TRAVEL);
    if (wristPastSafe.getAsBoolean()) {
      intake.setWantedState(WantedState.GRIP_CORAL);
      elevatorRun(L4);
      if (elevator.atGoal()) {
        elevatorWristRun(L4);
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

  private void scoreL4Auto(ScoringSide scoringSide) {
    wristRun(TRAVEL);
    if (wristPastSafe.getAsBoolean()) {
      elevatorRun(L4);
      intake.setWantedState(WantedState.GRIP_CORAL);
      if (elevator.atGoal()) {
        elevatorWristRun(L4);
      }

      if (isReadyToEjectInAutoPeriod()) {
        coralEject = true;
      }

      if (coralEject) {
        intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
      }
    }
  }

  private void moveAlgaeToNetPosition() {
    Rotation2d rotation = FieldConstants.isBlueAlliance() ? Rotation2d.k180deg : Rotation2d.kZero;
    // If barging on other side, uncomment
    /* if (Math.abs(
            RobotState.getInstance()
                .getRobotPoseFromSwerveDriveOdometry()
                .getRotation()
                .getDegrees())
        > 90) {
      rotation = Rotation2d.k180deg;
    } else {
      rotation = Rotation2d.kZero;
    } */

    drive.setTargetRotation(rotation);

    intake.setWantedState(Intake.WantedState.HOLD_ALGAE);
    drive.setTeleopVelocityCoefficient(0.4);

    if (drive.isAtDesiredRotation()) {
      elevatorWristRun(ALGAE_NET_PRE);
    }
  }

  private void moveAlgaeToProcessorPosition() {
    Rotation2d rotation =
        FieldConstants.isOnBlueAlliance(
                RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry())
            ? Rotation2d.kCW_90deg
            : Rotation2d.kCCW_90deg;
    drive.setTargetRotation(rotation);
  }

  private void scoreAlgaeNet() {
    drive.setTeleopVelocityCoefficient(0.0);
    if (drive.isAtDesiredRotation()) {
      elevatorWristRun(ALGAE_NET_POST);
    }
    if (wrist.getAngle().getRadians()
        > (ALGAE_NET_POST.wristAngle.minus(Rotation2d.fromDegrees(60))).getRadians())
      intake.setWantedState(Intake.WantedState.EJECT_ALGAE);
  }

  private void scoreAlgaeProcessor() {
    Rotation2d rotation =
        FieldConstants.isBlueAlliance() ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg;
    if (RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation().getDegrees()
        > 0) {
      rotation = Rotation2d.kCCW_90deg;
    } else {
      rotation = Rotation2d.kCW_90deg;
    }
    drive.setTargetRotation(rotation);
    elevatorWristRun(ALGAE_PROCESSOR);
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
                RobotState.getInstance().getClosestTagId(), scoringSide)
            : FieldConstants.getDesiredPointToDriveToForL1Scoring(
                RobotState.getInstance().getClosestTagId(), scoringSide);

    if (!hasDriveReachedIntermediatePoseForCoralScore) {
      Pose2d intermediatePose =
          FieldConstants.getDesiredIntermediateScoringPoseForCoral(
              RobotState.getInstance().getClosestTagId(), scoringSide);
      drive.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(intermediatePose, 3.0);
      if (drive.isAtDriveToPointSetpoint()) {
        hasDriveReachedIntermediatePoseForCoralScore = true;
      }
      Logger.recordOutput("Superstructure/DesiredPointToDriveTo", intermediatePose);
      return true;
    } else {
      drive.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(desiredPoseToDriveTo, 3.0);
      hasDriveToPointSetPointBeenSet = true;
      Logger.recordOutput("Superstructure/DesiredPointToDriveTo", desiredPoseToDriveTo);
      return true;
    }
  }

  public boolean driveToScoringPoseL4(ScoringSide scoringSide) {
    Pose2d desiredPoseToDriveTo =
        FieldConstants.getDesiredFinalScoringPoseForCoral(
            RobotState.getInstance().getClosestTagId(), scoringSide);
    Pose2d intermediatePose =
        FieldConstants.getDesiredIntermediateScoringPoseForCoral(
            RobotState.getInstance().getClosestTagId(), scoringSide);

    Pose2d preL4Pose =
        FieldConstants.getDesiredPointToDriveToForCoralScoring(
            RobotState.getInstance().getClosestTagId(), scoringSide, 0.15);

    if (!hasDriveReachedIntermediatePoseForCoralScore) {
      drive.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(intermediatePose, 3.0);
      if (drive.isAtDriveToPointSetpoint()) {
        hasDriveReachedIntermediatePoseForCoralScore = true;
      }
      Logger.recordOutput("Superstructure/DesiredPointToDriveTo", intermediatePose);
      return true;
    } else {
      if (elevator.atGoal()) {
        drive.setDesiredPoseForDriveToPointWithConstraints(desiredPoseToDriveTo, 0.5, 3.0);
        hasDriveToPointSetPointBeenSet = true;
        Logger.recordOutput("Superstructure/DesiredPointToDriveTo", desiredPoseToDriveTo);
      } else {
        drive.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(preL4Pose, 3.0);
        Logger.recordOutput("Superstructure/DesiredPointToDriveTo", preL4Pose);
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

    if (!hasDriveReachedIntermediatePoseForCoralScore) {
      Pose2d intermediatePose =
          FieldConstants.getDesiredIntermediateScoringPoseForCoral(id, scoringSide);
      drive.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(intermediatePose, 3.0);
      if (drive.isAtDriveToPointSetpoint()) {
        hasDriveReachedIntermediatePoseForCoralScore = true;
      }
      Logger.recordOutput("Superstructure/DesiredPointToDriveTo", intermediatePose);
      return true;
    } else {
      drive.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(desiredPoseToDriveTo, 3.0);
      hasDriveToPointSetPointBeenSet = true;
      Logger.recordOutput("Superstructure/DesiredPointToDriveTo", desiredPoseToDriveTo);
      return true;
    }
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

  public boolean hasCoral() {
    return intake.hasCoral();
  }

  public void setWantedSuperState(WantedSuperState superState) {
    this.wantedSuperState = superState;
  }

  public Command setStateCommand(WantedSuperState superState) {
    Command commandToReturn = new InstantCommand(() -> setWantedSuperState(superState));
    return commandToReturn;
  }

  public Command configureButtonBinding(
      WantedSuperState hasCoralCondition,
      WantedSuperState hasAlgaeCondition,
      WantedSuperState noPieceCondition) {
    return Commands.either(
        setStateCommand(hasAlgaeCondition),
        Commands.either(
            setStateCommand(hasCoralCondition),
            setStateCommand(noPieceCondition),
            intake::hasCoral),
        intake::hasAlgae);
  }

  public void elevatorWristRun(WristElevatorPoses.WristElevatorPose wristElevatorPose) {
    wrist.setWantedState(Wrist.WantedState.MOVE_TO_POSITION, wristElevatorPose.wristAngle);
    elevator.setWantedState(
        Elevator.WantedState.MOVE_TO_POSITION, wristElevatorPose.elevatorHeight);
  }

  public void elevatorRun(WristElevatorPoses.WristElevatorPose wristElevatorPose) {
    elevator.setWantedState(
        Elevator.WantedState.MOVE_TO_POSITION, wristElevatorPose.elevatorHeight);
  }

  public void wristRun(WristElevatorPoses.WristElevatorPose wristElevatorPose) {
    wrist.setWantedState(Wrist.WantedState.MOVE_TO_POSITION, wristElevatorPose.wristAngle);
  }

  public boolean allAtGoals() {
    return wrist.atGoal() && elevator.atGoal() && drive.isAtDriveToPointSetpoint();
  }

  public boolean wristElevatorAtGoal() {
    return wrist.atGoal() && elevator.atGoal();
  }
}
