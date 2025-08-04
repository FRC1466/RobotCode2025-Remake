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
import frc.robot.subsystems.sensors.CoralSensorIO;
import frc.robot.subsystems.sensors.CoralSensorIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * The Intake subsystem is responsible for managing the intake and outtake of game pieces,
 * specifically Coral and Algae. It controls two roller systems: an end effector and a star wheel,
 * and uses a Coral sensor to detect the presence of Coral. The subsystem operates based on a
 * state machine, transitioning between different states like intaking, holding, and ejecting
 * game pieces.
 */
public class Intake extends SubsystemBase {
  private final RollerSystemIO endEffectorIO;
  private final RollerSystemIOInputsAutoLogged endEffectorInputs =
      new RollerSystemIOInputsAutoLogged();
      
  private final RollerSystemIO starWheelIO;
  private final RollerSystemIOInputsAutoLogged starWheelInputs =
      new RollerSystemIOInputsAutoLogged();

  private final CoralSensorIO coralSensorIO;
  private final CoralSensorIOInputsAutoLogged coralSensorInputs =
      new CoralSensorIOInputsAutoLogged();
      
  /**
   * Represents the desired state of the Intake subsystem
   */
  public enum WantedState {
    INTAKE_CORAL,
    GRIP_CORAL,
    OUTTAKE_CORAL,
    OUTTAKE_CORAL_L1,
    BACKUP_CORAL,
    INTAKE_ALGAE,
    HOLD_ALGAE,
    HOLD_ALGAE_HARDER,
    EJECT_ALGAE,
    OFF
  }

  /**
   * Represents the actual, internal state of the Intake subsystem. This state is determined
   * by the subsystem's logic in the periodic loop, based on the wantedState and sensor feedback.
   */
  private enum SystemState {
    INTAKING_CORAL,
    GRIPPING_CORAL,
    OUTTAKING_CORAL,
    OUTTAKING_CORAL_L1,
    BACKING_UP_CORAL,
    INTAKING_ALGAE,
    HOLDING_ALGAE,
    HOLDING_ALGAE_HARDER,
    EJECTING_ALGAE,
    OFF
  }

  /**
   * Constructs a new Intake subsystem.
   *
   * @param endEffectorRollerSystemIO The I/O interface for the end effector roller system.
   * @param starWheelRollerSystemIO The I/O interface for the star wheel roller system.
   * @param coralSensorIO The I/O interface for the Coral distance sensor.
   */
  public Intake(
      RollerSystemIO endEffectorRollerSystemIO,
      RollerSystemIO starWheelRollerSystemIO,
      CoralSensorIO coralSensorIO) {
    this.endEffectorIO = endEffectorRollerSystemIO;
    this.starWheelIO = starWheelRollerSystemIO;
    this.coralSensorIO = coralSensorIO;
  }

  private WantedState wantedState = WantedState.OFF;
  private SystemState systemState = SystemState.OFF;
  
  private boolean hasAlgae = false;

  /**
   * This method is called periodically by the scheduler. It is the main workhorse of the subsystem.
   * It reads sensor inputs, updates the internal state machine, applies motor outputs based on the
   * current state, and logs all relevant data.
   */
  @Override
  public void periodic() {
    // Update and log all inputs
    endEffectorIO.updateInputs(endEffectorInputs);
    Logger.processInputs("Subsystems/Intake/EndEffectorRoller", endEffectorInputs);
    starWheelIO.updateInputs(starWheelInputs);
    Logger.processInputs("Subsystems/Intake/StarWheelRoller", starWheelInputs);
    coralSensorIO.updateInputs(coralSensorInputs);
    Logger.processInputs("Subsystems/Intake/CoralSensor", coralSensorInputs);

    // Handle state transitions and logic
    SystemState nextState = handleStateTransition();
    if (systemState != nextState) {
      systemState = nextState;
      // Reset hasAlgae flag unless transitioning to a state that holds algae
      if (systemState != SystemState.HOLDING_ALGAE
          && systemState != SystemState.HOLDING_ALGAE_HARDER) {
        hasAlgae = false;
      }
    }

    // Perform state-specific actions
    if (systemState == SystemState.INTAKING_ALGAE && !hasAlgae) {
      if (endEffectorInputs.data.torqueCurrentAmps() >= currentThresholdAlgaeIntake) {
        hasAlgae = true;
        systemState = SystemState.HOLDING_ALGAE_HARDER; // Transition to harder hold on detection
      }
    }

    applyState();

    // Log outputs
    Logger.recordOutput("Subsystems/Intake/SystemState", systemState);
    Logger.recordOutput("Subsystems/Intake/WantedState", wantedState);
    Logger.recordOutput("Subsystems/Intake/HasAlgae", hasAlgae);
  }

  /**
   * Determines the next system state based on the current wanted state and game piece possession.
   * This method encapsulates the primary logic for state transitions.
   *
   * @return The calculated next SystemState for the intake.
   */
  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case INTAKE_CORAL -> SystemState.INTAKING_CORAL;
      case GRIP_CORAL -> SystemState.GRIPPING_CORAL;
      case OUTTAKE_CORAL -> SystemState.OUTTAKING_CORAL;
      case OUTTAKE_CORAL_L1 -> SystemState.OUTTAKING_CORAL_L1;
      case BACKUP_CORAL -> SystemState.BACKING_UP_CORAL;
      case INTAKE_ALGAE -> hasAlgae ? SystemState.HOLDING_ALGAE : SystemState.INTAKING_ALGAE;
      case HOLD_ALGAE -> SystemState.HOLDING_ALGAE;
      case HOLD_ALGAE_HARDER -> SystemState.HOLDING_ALGAE_HARDER;
      case EJECT_ALGAE -> SystemState.EJECTING_ALGAE;
      default -> SystemState.OFF;
    };
  }

  /**
   * Applies the appropriate voltages to the end effector and star wheel motors based on the current
   * system state.
   */
  private void applyState() {
    RollerVoltages voltages;

    switch (systemState) {
      case INTAKING_CORAL -> voltages = coralIntake;
      case GRIPPING_CORAL -> voltages = coralGrip;
      case OUTTAKING_CORAL -> voltages = coralOuttake;
      case OUTTAKING_CORAL_L1 -> voltages = coralOuttakeL1;
      case BACKING_UP_CORAL -> voltages = coralBackup;
      case INTAKING_ALGAE -> voltages = algaeIntake;
      case HOLDING_ALGAE -> voltages = algaeHold;
      case HOLDING_ALGAE_HARDER -> voltages = algaeHold; // Or a different voltage if needed
      case EJECTING_ALGAE -> voltages = algaeEject;
      default -> voltages = new RollerVoltages(0.0, 0.0);
    }

    endEffectorIO.runVolts(voltages.endEffectorVoltage);
    starWheelIO.runVolts(voltages.starWheelVoltage);
  }

  /**
   * Checks if a Coral game piece is detected by the distance sensor.
   *
   * @return True if the sensor distance is less than the defined threshold, false otherwise.
   */
  public boolean hasCoral() {
    return coralSensorInputs.data.distanceMeters() < distanceThresholdCoralIntake;
  }

  /**
   * Checks if an Algae game piece has been successfully acquired. This is determined by monitoring
   * motor current during the intake process.
   *
   * @return True if Algae has been detected and is being held, false otherwise.
   */
  public boolean hasAlgae() {
    return hasAlgae;
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
