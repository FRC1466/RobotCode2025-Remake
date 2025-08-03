// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.constants.ElevatorConstants.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

/**
 * The Elevator subsystem is responsible for controlling the vertical movement of the robot's
 * elevator mechanism. It uses a state machine to manage its behavior, such as moving to a specific
 * position or remaining idle.
 */
public class Elevator extends SubsystemBase {
  /** Represents the desired state of the elevator, typically set by external commands. */
  public enum WantedState {
    IDLE,
    MOVE_TO_POSITION
  }

  /** Represents the internal operational state of the elevator. */
  private enum SystemState {
    IDLING,
    MOVING_TO_POSITION
  }

  /** Defines motion profiles with different velocity and acceleration constraints. */
  public enum ElevatorProfile {
    DEFAULT(() -> velocityConstraint.get(), () -> accelerationConstraint.get()),
    DOWN(() -> velocityConstraint.get(), () -> accelerationConstraintDown.get()),
    ALGAE(() -> velocityConstraintAlgae.get(), () -> accelerationConstraintAlgae.get());

    public final Supplier<Double> velocity;
    public final Supplier<Double> acceleration;

    ElevatorProfile(Supplier<Double> velocity, Supplier<Double> acceleration) {
      this.velocity = velocity;
      this.acceleration = acceleration;
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  @Getter private double goalPosition;

  @Getter @Setter private ElevatorProfile currentElevatorProfile = ElevatorProfile.DEFAULT;
  private ElevatorProfile lastProfile = ElevatorProfile.DEFAULT;

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.goalPosition = stowed.get(); // Default to stowed position on startup
  }

  /**
   * Sets the desired state for the elevator. This is the primary way commands interact with the
   * subsystem's state machine.
   *
   * @param wantedState The desired state.
   */
  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  /**
   * Sets the desired state for the elevator and the target position to move to.
   *
   * @param wantedState The desired state.
   * @param goalPosition The target position in meters to move the elevator to.
   */
  public void setWantedState(WantedState wantedState, double goalPositionMeters) {
    this.wantedState = wantedState;
    setGoalPosition(goalPositionMeters);
  }

  /**
   * Commands the elevator to move to a specific position. Does NOT change the wanted state
   *
   * @param goal The target position in meters.
   */
  private void setGoalPosition(double goal) {
    this.goalPosition = goal;
  }

  /**
   * Checks if the elevator is at its target position within a specified tolerance.
   *
   * @return True if the elevator is at the goal, false otherwise.
   */
  public boolean atGoal() {
    return MathUtil.isNear(getPosition(), getGoalPosition(), Units.inchesToMeters(5.0));
  }

  /**
   * Sets the neutral mode for the elevator motors (e.g., Brake or Coast).
   *
   * @param neutralModeValue The neutral mode to set.
   */
  public void setNeutralMode(NeutralModeValue neutralModeValue) {
    io.setNeutralMode(neutralModeValue);
  }

  /**
   * Gets the current position of the elevator.
   *
   * @return The current position in meters.
   */
  public double getPosition() {
    return inputs.elevatorPositionMeters;
  }

  /**
   * Gets the current velocity of the elevator.
   *
   * @return The current velocity in meters per second.
   */
  public double getVelocity() {
    return inputs.elevatorVelocityMetersPerSec;
  }

  /**
   * Gets the current acceleration of the elevator.
   *
   * @return The current acceleration in meters per second squared.
   */
  public double getAcceleration() {
    return inputs.elevatorAccelerationMetersPerSecSquared;
  }

  @Override
  public void periodic() {
    // Update and log inputs from the hardware layer
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    if (ElevatorConstants.kP.hasChanged(hashCode())
        || ElevatorConstants.kI.hasChanged(hashCode())
        || ElevatorConstants.kD.hasChanged(hashCode())) {
      // Reconfigure the elevator if PID constants have changed
      io.setPID(kP.get(), kI.get(), kD.get());
    }

    // Update the motion profile if it has changed
    updateMotionProfile();

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
        io.setTargetPosition(goalPosition);
        break;
    }
  }

  /**
   * Checks if the current motion profile has been changed and applies the new constraints to the
   * hardware layer if necessary.
   */
  private void updateMotionProfile() {
    if (currentElevatorProfile != lastProfile) {
      io.setMotionProfileConstraints(currentElevatorProfile);
      lastProfile = currentElevatorProfile;
    }
  }

  /** Logs the essential state of the subsystem to AdvantageKit Logger. */
  private void logState() {
    Logger.recordOutput("Elevator/SystemState", systemState.name());
    Logger.recordOutput("Elevator/WantedState", wantedState.name());
    Logger.recordOutput("Elevator/MotionProfile", currentElevatorProfile.name());
    Logger.recordOutput("Elevator/GoalPositionMeters", goalPosition);
    Logger.recordOutput("Elevator/AtGoal", atGoal());
    Logger.recordOutput("Elevator/PositionMeters", getPosition());
    Logger.recordOutput("Elevator/VelocityMetersPerSec", getVelocity());
  }
}
