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
import frc.robot.util.LoggedTracer;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

/**
 * Controls the rotational movement of the robot's wrist with continuous angle tracking, smart
 * setpoint selection, and directional movement forcing.
 */
public class Wrist extends SubsystemBase {

  public enum WantedState {
    IDLE,
    MOVE_TO_POSITION
  }

  private enum SystemState {
    IDLING,
    MOVING_TO_POSITION
  }

  /** Direction mode for motion selection. */
  public enum ForceDirection {
    SHORTEST,
    CLOCKWISE,
    COUNTERCLOCKWISE
  }

  private static final double motorMinRadians = Units.degreesToRadians(-540.0);
  private static final double motorMaxRadians = Units.degreesToRadians(540.0);
  private static final int searchTurns = 3;
  private static final double atGoalToleranceRadians = Units.degreesToRadians(5);
  private static final double forceDirectionToleranceRadians = Units.degreesToRadians(10);

  private DoubleSupplier elevatorHeightMeters = () -> 0.0;

  private double maxForcedTravelRadians = motorMaxRadians - motorMinRadians;

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  @Getter private Rotation2d goalAngle;
  @Setter @Getter private ForceDirection forceDirection = ForceDirection.SHORTEST;
  @Setter private boolean exactAngle = false;

  public Wrist(WristIO io, DoubleSupplier elevatorHeightMeters) {
    this.io = io;
    this.goalAngle = Rotation2d.fromRadians(stowedPosition.get());
    this.elevatorHeightMeters = elevatorHeightMeters;
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public void setWantedState(WantedState wantedState, Rotation2d goalAngle) {
    this.wantedState = wantedState;
    setGoalPosition(goalAngle);
  }

  /** Set the target wrist angle without changing wanted state. */
  private void setGoalPosition(Rotation2d goal) {
    this.goalAngle = goal;
  }

  public boolean atGoal() {
    return MathUtil.isNear(
        getAngle().getRadians(),
        computeSmartSetpoint(
            getAngle().getRadians(), goalAngle.getRadians(), forceDirection, exactAngle),
        atGoalToleranceRadians);
  }

  // Clamp the desired angle to the closest safe angle based on elevator height
  private double clampToSafeAngle(double desiredRadians) {
    double elevatorHeight = elevatorHeightMeters.getAsDouble();

    // If elevatorHeight >= max required (0.82), all angles are safe
    if (elevatorHeight >= 0.8) {
      return desiredRadians;
    }

    // Search for safe angle intervals numerically
    // Step in degrees for precision
    final int steps = 360;
    double stepRadians = 2.0 * Math.PI / steps;

    // Find all safe angles
    boolean[] safeAngles = new boolean[steps];
    for (int i = 0; i < steps; i++) {
      double angle = i * stepRadians;
      double safeHeight = minHeightForAngle(Math.toDegrees(angle));
      safeAngles[i] = (elevatorHeight >= safeHeight);
    }

    // Find continuous safe segments; for simplicity, assume one continuous safe region around 0/360
    // degrees
    // We find closest safe angle to desiredRadians

    // Convert desiredRadians to index
    int desiredIndex =
        (int) Math.round((normalizeAngle(desiredRadians) / (2.0 * Math.PI)) * steps) % steps;

    // Search outward from desiredIndex for closest safe angle
    for (int offset = 0; offset <= steps / 2; offset++) {
      int plusIndex = (desiredIndex + offset) % steps;
      if (safeAngles[plusIndex]) {
        return plusIndex * stepRadians;
      }
      int minusIndex = (desiredIndex - offset + steps) % steps;
      if (safeAngles[minusIndex]) {
        return minusIndex * stepRadians;
      }
    }

    // If no safe angle found (extreme case), just return desiredRadians
    return desiredRadians;
  }

  private double minHeightForAngle(double angleDegrees) {
    double cosVal = Math.cos(Math.toRadians(angleDegrees));
    return 0.11 * cosVal * cosVal - 0.41 * cosVal + 0.3;
  }

  public void setNeutralMode(NeutralModeValue neutralModeValue) {
    io.setNeutralMode(neutralModeValue);
  }

  public Rotation2d getAngle() {
    return inputs.wristAngle;
  }

  public double getVelocity() {
    return inputs.wristAngularVelocityRadPerSec;
  }

  public double getAcceleration() {
    return inputs.wristAngularAccelerationRadPerSecSquared;
  }

  /** Return to normal shortest-path behavior. */
  public void clearForceDirection() {
    this.forceDirection = ForceDirection.SHORTEST;
  }

  /** Set maximum travel distance in forced-direction mode. */
  public void setMaxForcedTravelRadians(double maxTravelRadians) {
    if (maxTravelRadians < 0) {
      throw new IllegalArgumentException("maxTravelRadians must be >= 0");
    }
    this.maxForcedTravelRadians = maxTravelRadians;
  }

  /** Set maximum travel distance in forced-direction mode (degrees). */
  public void setMaxForcedTravelDegrees(double degrees) {
    setMaxForcedTravelRadians(Units.degreesToRadians(Math.abs(degrees)));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Subsystems/Wrist", inputs);

    if (WristConstants.kP.hasChanged(hashCode())
        || WristConstants.kI.hasChanged(hashCode())
        || WristConstants.kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get());
    }

    this.systemState = handleStateTransitions();
    applyStates();
    logState();
    LoggedTracer.record("Wrist");
  }

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

  private void applyStates() {
    switch (systemState) {
      case IDLING:
        io.setDutyCycle(0.0);
        break;
      case MOVING_TO_POSITION:
        double currentRadians = getAngle().getRadians();
        double desiredRadians = goalAngle.getRadians();
        double chosenSetpoint =
            computeSmartSetpoint(currentRadians, desiredRadians, forceDirection, exactAngle);
        io.setTargetAngle(Rotation2d.fromRadians(chosenSetpoint));
        break;
    }
  }

  private void logState() {
    Logger.recordOutput("Subsystems/Wrist/SystemState", systemState.name());
    Logger.recordOutput("Subsystems/Wrist/WantedState", wantedState.name());
    Logger.recordOutput("Subsystems/Wrist/GoalAngle", goalAngle);
    Logger.recordOutput("Subsystems/Wrist/AtGoal", atGoal());
    Logger.recordOutput("Subsystems/Wrist/Angle", getAngle());
    Logger.recordOutput("Subsystems/Wrist/VelocityRadPerSec", getVelocity());
    Logger.recordOutput(
        "Subsystems/Wrist/OptimizedGoalAngle",
        Rotation2d.fromRadians(
            computeSmartSetpoint(
                getAngle().getRadians(), goalAngle.getRadians(), forceDirection, exactAngle)));
    Logger.recordOutput("Subsystems/Wrist/Direction", forceDirection);
  }

  /**
   * Compute optimal setpoint considering motor limits and directional constraints. Chooses shortest
   * path normally, or enforces directional movement when requested.
   */
  private double computeSmartSetpoint(
      double currentRadians, double desiredRadians, ForceDirection direction, boolean exactAngle) {
    desiredRadians = clampToSafeAngle(desiredRadians);
    if (exactAngle) {
      return desiredRadians;
    }

    double desiredWrapped = normalizeAngle(desiredRadians);
    double currentWrapped = normalizeAngle(currentRadians);

    // Calculate shortest angular difference
    double delta = desiredWrapped - currentWrapped;
    if (delta > Math.PI) delta -= 2.0 * Math.PI;
    if (delta <= -Math.PI) delta += 2.0 * Math.PI;

    if (Math.abs(delta) <= forceDirectionToleranceRadians) {
      direction = ForceDirection.SHORTEST;
    }

    if (direction == ForceDirection.SHORTEST) {
      double candidate = currentRadians + delta;
      candidate = wrapIntoBounds(candidate);
      return clampToBounds(candidate);
    }

    // Forced direction path
    int directionSign = (direction == ForceDirection.CLOCKWISE) ? 1 : -1;
    double bestCandidate = Double.NaN;
    double bestDistance = Double.POSITIVE_INFINITY;

    for (int n = -searchTurns; n <= searchTurns; n++) {
      double candidate = currentRadians + delta + n * 2.0 * Math.PI;

      if (candidate < motorMinRadians || candidate > motorMaxRadians) continue;

      double movement = candidate - currentRadians;

      // Must move in forced direction
      if (directionSign > 0 && movement < -1e-9) continue;
      if (directionSign < 0 && movement > 1e-9) continue;

      double absMovement = Math.abs(movement);
      if (absMovement > maxForcedTravelRadians + 1e-9) continue;

      if (absMovement < bestDistance) {
        bestDistance = absMovement;
        bestCandidate = candidate;
      }
    }

    if (!Double.isNaN(bestCandidate)) {
      return bestCandidate;
    }

    // Fallback: move to nearest reachable position in forced direction
    if (directionSign > 0) {
      double reachable = Math.min(currentRadians + maxForcedTravelRadians, motorMaxRadians);
      return Math.max(reachable, currentRadians);
    } else {
      double reachable = Math.max(currentRadians - maxForcedTravelRadians, motorMinRadians);
      return Math.min(reachable, currentRadians);
    }
  }

  /** Try to shift angle by ±2π to fit within motor bounds. */
  private double wrapIntoBounds(double angle) {
    if (angle >= motorMinRadians && angle <= motorMaxRadians) return angle;

    // Try positive shifts
    double attempt = angle;
    for (int i = 0; i <= searchTurns; i++) {
      if (attempt >= motorMinRadians && attempt <= motorMaxRadians) return attempt;
      attempt += 2.0 * Math.PI;
    }

    // Try negative shifts
    attempt = angle;
    for (int i = 0; i <= searchTurns; i++) {
      if (attempt >= motorMinRadians && attempt <= motorMaxRadians) return attempt;
      attempt -= 2.0 * Math.PI;
    }

    return angle;
  }

  /** Clamp angle to motor bounds as last resort. */
  private double clampToBounds(double angle) {
    return MathUtil.clamp(angle, motorMinRadians, motorMaxRadians);
  }

  /** Normalize angle to [0, 2π). */
  private static double normalizeAngle(double angleRadians) {
    double normalized = angleRadians % (2.0 * Math.PI);
    return normalized < 0 ? normalized + 2.0 * Math.PI : normalized;
  }
}
