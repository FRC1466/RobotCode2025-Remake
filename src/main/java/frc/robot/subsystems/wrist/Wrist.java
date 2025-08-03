// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.wrist;

import static frc.robot.constants.WristConstants.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.WristConstants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/**
 * The Wrist subsystem is responsible for controlling the rotational movement of the robot's wrist
 * mechanism. It uses a state machine to manage its behavior, such as moving to a specific position
 * or remaining idle.
 */
public class Wrist extends SubsystemBase {
  /** Represents the desired state of the wrist, typically set by external commands. */
  public enum WantedState {
    IDLE,
    MOVE_TO_POSITION
  }

  /** Represents the internal operational state of the wrist. */
  private enum SystemState {
    IDLING,
    MOVING_TO_POSITION
  }

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  @Getter private Rotation2d goalAngle;

  public Wrist(WristIO io) {
    this.io = io;
    this.goalAngle =
        Rotation2d.fromRadians(stowedPosition.get()); // Default to stowed position on startup
  }

  /**
   * Sets the desired state for the wrist. This is the primary way commands interact with the
   * subsystem's state machine.
   *
   * @param wantedState The desired state.
   */
  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  /**
   * Sets the desired state for the wrist and the target position to move to.
   *
   * @param wantedState The desired state.
   * @param goalAngle The target position in meters to move the wrist to.
   */
  public void setWantedState(WantedState wantedState, Rotation2d goalAngle) {
    this.wantedState = wantedState;
    setGoalPosition(goalAngle);
  }

  /**
   * Commands the wrist to move to a specific position. Does NOT change the wanted state
   *
   * @param goal The target angle.
   */
  private void setGoalPosition(Rotation2d goal) {
    this.goalAngle = goal;
  }

  /**
   * Checks if the wrist is at its target position within a specified tolerance.
   *
   * @return True if the wrist is at the goal, false otherwise.
   */
  public boolean atGoal() {
    return MathUtil.isNear(
        getAngle().getRadians(), getGoalAngle().getRadians(), Units.inchesToMeters(5.0));
  }

  /**
   * Sets the neutral mode for the wrist motors (e.g., Brake or Coast).
   *
   * @param neutralModeValue The neutral mode to set.
   */
  public void setNeutralMode(NeutralModeValue neutralModeValue) {
    io.setNeutralMode(neutralModeValue);
  }

  /**
   * Gets the current position of the wrist.
   *
   * @return The current position in meters.
   */
  public Rotation2d getAngle() {
    return inputs.wristAngle;
  }

  /**
   * Gets the current velocity of the wrist.
   *
   * @return The current velocity in meters per second.
   */
  public double getVelocity() {
    return inputs.wristAngularVelocityRadPerSec;
  }

  /**
   * Gets the current acceleration of the wrist.
   *
   * @return The current acceleration in meters per second squared.
   */
  public double getAcceleration() {
    return inputs.wristAngularAccelerationRadPerSecSquared;
  }

  @Override
  public void periodic() {
    // Update and log inputs from the hardware layer
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);

    if (WristConstants.kP.hasChanged(hashCode())
        || WristConstants.kI.hasChanged(hashCode())
        || WristConstants.kD.hasChanged(hashCode())) {
      // Reconfigure the wrist if PID constants have changed
      io.setPID(kP.get(), kI.get(), kD.get());
    }

    // Run the state machine logic
    this.systemState = handleStateTransitions();
    applyStates();

    // Log subsystem state for debugging and analysis
    logState();
  }

  /**
   * Determines the current system state based on the wanted state. This is the "transition" part of
   * the state machine.
   */
  private SystemState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return SystemState.IDLING;
      case MOVE_TO_POSITION:
        return SystemState.MOVING_TO_POSITION;
      default:
        return SystemState.IDLING;
    }
  }

  /**
   * Executes actions based on the current system state. This is the "action" part of the state
   * machine.
   */
  private void applyStates() {
    switch (systemState) {
      case IDLING:
        io.setDutyCycle(0.0);
        break;
      case MOVING_TO_POSITION:
        io.setTargetAngle(goalAngle);
        break;
    }
  }

  /** Logs the essential state of the subsystem to AdvantageKit Logger. */
  private void logState() {
    Logger.recordOutput("Subsystems/Wrist/SystemState", systemState.name());
    Logger.recordOutput("Subsystems/Wrist/WantedState", wantedState.name());
    Logger.recordOutput("Subsystems/Wrist/GoalPositionMeters", goalAngle);
    Logger.recordOutput("Subsystems/Wrist/AtGoal", atGoal());
    Logger.recordOutput("Subsystems/Wrist/PositionMeters", getAngle());
    Logger.recordOutput("Subsystems/Wrist/VelocityMetersPerSec", getVelocity());
  }
}
