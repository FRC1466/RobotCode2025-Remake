// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

/**
 * Simplified Choreographer: - Only four choreographies: STOPPED, DEFAULT_STATE,
 * INTAKE_CORAL_FROM_STATION, SCORE_L1 - Intake is just a single roller: ON during SCORE_L1
 * (outtaking), OFF otherwise.
 */
public class Choreographer extends SubsystemBase {

  public enum WantedChoreography {
    STOPPED,
    DEFAULT_STATE,
    INTAKE_CORAL_FROM_STATION,
    SCORE_L1
  }

  private enum CurrentChoreography {
    STOPPED,
    DEFAULT_STATE,
    INTAKE_CORAL_FROM_STATION,
    SCORE_L1
  }

  private final Intake intake;

  private WantedChoreography wantedChoreography = WantedChoreography.STOPPED;
  private CurrentChoreography currentChoreography = CurrentChoreography.STOPPED;
  private CurrentChoreography previousChoreography = CurrentChoreography.STOPPED;

  public Choreographer(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void periodic() {
    previousChoreography = currentChoreography;
    currentChoreography = computeChoreography();

    applyChoreography();
    Logger.recordOutput("Choreographer/Wanted", wantedChoreography);
    Logger.recordOutput("Choreographer/Current", currentChoreography);
    Logger.recordOutput("Choreographer/Previous", previousChoreography);
  }

  private CurrentChoreography computeChoreography() {
    // Because there are no superstructure interactions, it is direct mapping
    return switch (wantedChoreography) {
      case STOPPED -> CurrentChoreography.STOPPED;
      case DEFAULT_STATE -> CurrentChoreography.DEFAULT_STATE;
      case INTAKE_CORAL_FROM_STATION -> CurrentChoreography.INTAKE_CORAL_FROM_STATION;
      case SCORE_L1 -> CurrentChoreography.SCORE_L1;
    };
  }

  private void applyChoreography() {
    switch (currentChoreography) {
      case STOPPED, DEFAULT_STATE -> intake.setWantedState(Intake.WantedState.OFF);
      case INTAKE_CORAL_FROM_STATION -> {
        // Currently treating everything except scoring as OFF.
        // This could later be changed if we have a second pair of rollers or we notice the coral
        // falling out
        intake.setWantedState(Intake.WantedState.OFF);
      }
      case SCORE_L1 -> intake.setWantedState(Intake.WantedState.MOVE_ROLLER); // Outtake/on
    }
  }

  public void setWantedChoreography(WantedChoreography choreography) {
    this.wantedChoreography = choreography;
  }

  public Command setWantedChoreographyCommand(WantedChoreography choreography) {
    return new InstantCommand(() -> setWantedChoreography(choreography));
  }

  public void defaultState() {
    setWantedChoreography(WantedChoreography.DEFAULT_STATE);
  }

  public void intakeFromStation() {
    setWantedChoreography(WantedChoreography.INTAKE_CORAL_FROM_STATION);
  }

  public void scoreL1() {
    setWantedChoreography(WantedChoreography.SCORE_L1);
  }

  public Command defaultStateCommand() {
    return new InstantCommand(() -> setWantedChoreography(WantedChoreography.DEFAULT_STATE));
  }

  public Command intakeFromStationCommand() {
    return new InstantCommand(
        () -> setWantedChoreography(WantedChoreography.INTAKE_CORAL_FROM_STATION));
  }

  public Command scoreL1Command() {
    return new InstantCommand(() -> setWantedChoreography(WantedChoreography.SCORE_L1));
  }
}
