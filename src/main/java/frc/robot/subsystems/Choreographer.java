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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Position;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Choreographer extends SubsystemBase {
  private final Drive drive;
  private final Intake intake;
  private final Vision vision;

  private static final double defaultTeleopTranslationCoefficient = 1.0;

  // Debouncers
  private final Debouncer simCoralDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kRising);

  public enum WantedChoreography {
    STOPPED,
    DEFAULT_STATE,
    INTAKE_CORAL_FROM_STATION,
    SCORE_L1
  }

  private enum CurrentChoreography {
    STOPPED,
    INTAKE_CORAL_FROM_STATION,
    SCORE_L1
  }

  private WantedChoreography wantedChoreography = WantedChoreography.STOPPED;
  private CurrentChoreography currentChoreography = CurrentChoreography.STOPPED;
  private CurrentChoreography previousChoreography;

  private boolean coralEject = false;

  public Choreographer(
      Drive drive,
      Intake intake,
      Vision vision) {
    this.drive = drive;
    this.intake = intake;
    this.vision = vision;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Choreographer/Wanted", wantedChoreography);
    Logger.recordOutput("Choreographer/Current", currentChoreography);
    Logger.recordOutput("Choreographer/Previous", previousChoreography);

    currentChoreography = computeChoreography();
    applyChoreography();
  }

  private CurrentChoreography computeChoreography() {
    previousChoreography = currentChoreography;
    switch (wantedChoreography) {
      default:
        currentChoreography = CurrentChoreography.STOPPED;
        break;
    }
    return currentChoreography;
  }

  private void applyChoreography() {
    if (previousChoreography != currentChoreography) {
      resetDebouncers();
    }

    switch (currentChoreography) {
      case INTAKE_CORAL_FROM_STATION:
        intakeCoralFromStation();
        break;
      case SCORE_L1:
        scoreL1Teleop();
        break;
      case STOPPED:
        stopped();
        break;
    }
  }

  public void resetDebouncers() {
    simCoralDebouncer.calculate(false);
  }

  private void home() {}

  private void stopped() {
    intake.setWantedState(Intake.WantedState.OFF);
  }

  private void intakeCoralFromStation() {}

  private void scoreL1Teleop() {}

}