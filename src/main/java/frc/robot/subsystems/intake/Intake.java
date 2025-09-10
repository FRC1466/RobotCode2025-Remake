// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOInputsAutoLogged;
import frc.robot.util.LoggedTracer;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/**
 * Bare-bones Intake subsystem controlling a single roller. Supports two states: ON (moving roller)
 * and OFF (stopped). No game piece sensing or tracking is implemented. Designed to be easily
 * expanded later.
 */
public class Intake extends SubsystemBase {
  private final RollerSystemIO rollerIO;
  private final RollerSystemIOInputsAutoLogged rollerInputs = new RollerSystemIOInputsAutoLogged();

  /** Represents the desired state of the Intake subsystem */
  public enum WantedState {
    MOVE_ROLLER,
    OFF
  }

  /**
   * Represents the actual, internal state of the Intake subsystem. This state is determined by the
   * subsystem's logic in the periodic loop, based on the wantedState and sensor feedback.
   */
  private enum SystemState {
    MOVING_ROLLER,
    OFF
  }

  /**
   * Constructs a new Intake subsystem.
   *
   * @param rollerSystemIO The I/O interface for the end effector roller system.
   */
  public Intake(RollerSystemIO rollerIO) {
    this.rollerIO = rollerIO;
  }

  @Getter private WantedState wantedState = WantedState.OFF;
  @Getter private SystemState systemState = SystemState.OFF;

  @Override
  public void periodic() {
    // Update and log all inputs
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Subsystems/Intake/Roller", rollerInputs);

    // Handle state transitions and logic
    systemState = handleStateTransition();

    applyState();

    // Log outputs
    Logger.recordOutput("Subsystems/Intake/SystemState", systemState);
    Logger.recordOutput("Subsystems/Intake/WantedState", wantedState);

    LoggedTracer.record("Intake");
  }

  /**
   * Determines the next system state based on the current wanted state and game piece possession.
   * This method encapsulates the primary logic for state transitions.
   *
   * @return The calculated next SystemState for the intake.
   */
  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case MOVE_ROLLER -> SystemState.MOVING_ROLLER;
      default -> SystemState.OFF;
    };
  }

  /**
   * Applies the appropriate voltages to the end effector and star wheel motors based on the current
   * system state.
   */
  private void applyState() {
    double voltage;

    switch (systemState) {
      case MOVING_ROLLER -> voltage = coralOuttake;
      default -> voltage = 0;
    }

    rollerIO.runVolts(voltage);
  }

  /**
   * Sets the desired state for the intake subsystem. This is typically called by a command in
   * response to driver input.
   *
   * @param wantedState The desired state to which the subsystem should transition.
   */
  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }
}
