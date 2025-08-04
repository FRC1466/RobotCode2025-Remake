// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.ElevatorConstants.*;
import static frc.robot.constants.IntakeConstants.*;
import static frc.robot.constants.SuperstructureConstants.*;
import static frc.robot.constants.WristConstants.*;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SuperstructureConstants.ScoringSide;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.overridePublisher.OverridePublisher;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.wrist.Wrist;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Intake intake;
  private final Wrist wrist;
  private final Elevator elevator;
  private final OverridePublisher overrides;
  private final Vision vision;

  private final CommandXboxController controller = new CommandXboxController(0);

  private static final double defaultTeleopTranslationCoeffecient = 1.0;

  private final Timer coralL1TopTimer = new Timer();

  public enum WantedSuperState {
    HOME,
    STOPPED,
    DEFAULT_STATE,
    INTAKE_CORAL_FROM_STATION,
    SCORE_L1,
    SCORE_LEFT_L2,
    SCORE_LEFT_L3,
    SCORE_LEFT_L4,
    SCORE_RIGHT_L2,
    SCORE_RIGHT_L3,
    SCORE_RIGHT_L4,
    MANUAL_L4,
    MANUAL_L3,
    MANUAL_L2,
    MANUAL_L1,
    INTAKE_ALGAE_REEF,
    INTAKE_ALGAE_GROUND,
    INTAKE_ALGAE_ICE_CREAM,
    MOVE_ALGAE_TO_NET_POSITION,
    SCORE_ALGAE_IN_NET,
    MOVE_ALGAE_TO_PROCESSOR_POSITION,
    SCORE_ALGAE_IN_PROCESSOR
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
    SCORE_TELEOP_L1,
    SCORE_LEFT_TELEOP_L2,
    SCORE_LEFT_TELEOP_L3,
    SCORE_LEFT_TELEOP_L4,
    SCORE_RIGHT_TELEOP_L2,
    SCORE_RIGHT_TELEOP_L3,
    SCORE_RIGHT_TELEOP_L4,
    SCORE_AUTO_L1,
    SCORE_LEFT_AUTO_L2,
    SCORE_LEFT_AUTO_L3,
    SCORE_LEFT_AUTO_L4,
    SCORE_RIGHT_AUTO_L2,
    SCORE_RIGHT_AUTO_L3,
    SCORE_RIGHT_AUTO_L4,
    MANUAL_L4,
    MANUAL_L3,
    MANUAL_L2,
    MANUAL_L1,
    INTAKE_ALGAE_REEF,
    INTAKE_ALGAE_GROUND,
    INTAKE_ALGAE_ICE_CREAM,
    MOVE_ALGAE_TO_NET_POSITION,
    SCORE_ALGAE_IN_NET,
    MOVE_ALGAE_TO_PROCESSOR_POSITION,
    SCORE_ALGAE_IN_PROCESSOR,
  }

  private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
  private CurrentSuperState previousSuperState;

  private boolean hasDriveToPointSetPointBeenSet = false;

  private boolean coralEject = false;

  private boolean algaeEject = false;

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
      case SCORE_L1:
        currentSuperState =
            DriverStation.isAutonomous()
                ? CurrentSuperState.SCORE_AUTO_L1
                : CurrentSuperState.SCORE_TELEOP_L1;
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
      case MANUAL_L1:
        currentSuperState = CurrentSuperState.MANUAL_L1;
        break;
      case MANUAL_L2:
        currentSuperState = CurrentSuperState.MANUAL_L2;
        break;
      case MANUAL_L3:
        currentSuperState = CurrentSuperState.MANUAL_L3;
        break;
      case MANUAL_L4:
        currentSuperState = CurrentSuperState.MANUAL_L4;
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
      case SCORE_TELEOP_L1:
        scoreL1Teleop();
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
      case SCORE_RIGHT_TELEOP_L2:
        scoreL2Teleop(ScoringSide.RIGHT);
        break;
      case SCORE_RIGHT_TELEOP_L3:
        scoreL3Teleop(ScoringSide.RIGHT);
        break;
      case SCORE_RIGHT_TELEOP_L4:
        scoreL4Teleop(ScoringSide.RIGHT);
        break;
      case SCORE_AUTO_L1:
        scoreL1Auto();
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
      case SCORE_RIGHT_AUTO_L2:
        scoreL2Auto(ScoringSide.RIGHT);
        break;
      case SCORE_RIGHT_AUTO_L3:
        scoreL3Auto(ScoringSide.RIGHT);
        break;
      case SCORE_RIGHT_AUTO_L4:
        scoreL4Auto(ScoringSide.RIGHT);
        break;
      case MANUAL_L4:
        ejectL4();
        break;
      case MANUAL_L3:
        ejectL3();
        break;
      case MANUAL_L2:
        ejectL2();
        break;
      case MANUAL_L1:
        ejectL1();
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
      case STOPPED:
        stopped();
        break;
    }
  }

  private void home() {}

  private void stopped() {}

  private void holdingAlgae() {}

  private void holdingCoral() {}

  private void holdingCoralAuto() {}

  private void noPiece() {}

  private void noPieceAuto() {}

  private void intakeCoralFromStation() {}

  private void intakeAlgaeFromReef() {}

  private void intakeAlgaeFromGround() {}

  private void intakeAlgaeIceCream() {}

  private void ejectL1() {}

  private void ejectL2() {}

  private void ejectL3() {}

  private void ejectL4() {}

  private void scoreL1Teleop() {}

  private void scoreL1Auto() {}

  private void scoreL2Teleop(ScoringSide scoringSide) {}

  private void scoreL2Auto(ScoringSide scoringSide) {}

  private void scoreL3Teleop(ScoringSide scoringSide) {}

  private void scoreL3Auto(ScoringSide scoringSide) {}

  private void scoreL4Teleop(ScoringSide scoringSide) {}

  private void scoreL4Auto(ScoringSide scoringSide) {}

  private void moveAlgaeToNetPosition() {}

  private void moveAlgaeToProcessorPosition() {}

  private void scoreAlgaeNet() {}

  private void scoreAlgaeProcessor() {}

  public boolean isReadyToEjectInAutoPeriod() {
    return false;
  }

  public boolean isReadyToEjectInTeleopPeriod() {
    return false;
  }

  public boolean driveToScoringPoseAndReturnIfObservationIsPresent(
      ScoringSide scoringSide,
      boolean isScoringL4OverTheBackAndArmIsNotAtPose,
      boolean isScoringReefCoral) {
    return false;
  }

  public boolean driveToScoringPoseAndReturnIfObservationIsPresent(ScoringSide scoringSide) {
    return false;
  }

  public boolean driveToScoringPoseAndReturnIfObservationIsPresent(
      ScoringSide scoringSide,
      boolean isScoringL4OverTheBackAndArmIsNotAtPose,
      boolean isScoringReefCoral,
      boolean isScoringBase) {
    return false;
  }

  @AutoLogOutput
  public Boolean reefTagVisible() {
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

  public boolean hasCollectedPieceInAuto() {
    return false;
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
    if (intake.hasAlgae()) {
      return setStateCommand(hasAlgaeCondition);
    } else if (intake.hasCoral()) {
      return setStateCommand(hasCoralCondition);
    } else {
      return setStateCommand(noPieceCondition);
    }
  }
}
