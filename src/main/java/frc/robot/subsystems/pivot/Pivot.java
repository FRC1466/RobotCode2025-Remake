// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivot;

import static frc.robot.constants.PivotConstants.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PivotConstants;
import frc.robot.util.LoggedTracer;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/** The Pivot subsystem controls the slapdown arm rotation using a simple state machine. */
public class Pivot extends SubsystemBase {
  public enum WantedState {
    IDLE,
    MOVE_TO_POSITION
  }

  private enum SystemState {
    IDLING,
    MOVING_TO_POSITION
  }

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  @Getter private Rotation2d goalAngle;

  private static final double kAngleToleranceRad = Units.degreesToRadians(2.0);

  public Pivot(PivotIO io) {
    this.io = io;
    this.goalAngle = Rotation2d.fromRadians(stowedPosition.get());
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public void setWantedState(WantedState wantedState, Rotation2d goalAngle) {
    this.wantedState = wantedState;
    setGoalPosition(goalAngle);
  }

  private void setGoalPosition(Rotation2d goal) {
    this.goalAngle = goal;
  }

  public boolean atGoal() {
    return MathUtil.isNear(
        getAngle().getRadians(), getGoalAngle().getRadians(), kAngleToleranceRad);
  }

  public boolean atGoal(Rotation2d tolerance) {
    return MathUtil.isNear(
        getAngle().getRadians(), getGoalAngle().getRadians(), tolerance.getRadians());
  }

  public void setNeutralMode(NeutralModeValue neutralModeValue) {
    io.setNeutralMode(neutralModeValue);
  }

  public Rotation2d getAngle() {
    return inputs.data.angle();
  }

  public double getVelocity() {
    return inputs.data.angularVelocityRotPerSec();
  }

  public double getAcceleration() {
    return inputs.data.angularAccelerationRadPerSecSquared();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Subsystems/Pivot", inputs);

    if (PivotConstants.kP.hasChanged(hashCode())
        || PivotConstants.kI.hasChanged(hashCode())
        || PivotConstants.kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get());
    }

    systemState = handleStateTransitions();
    applyStates();
    logState();
    LoggedTracer.record("Pivot");
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
        io.setTargetAngle(goalAngle);
        break;
    }
  }

  public void resetAngle(Rotation2d angle) {
    io.resetAngle(angle);
  }

  private void logState() {
    Logger.recordOutput("Subsystems/Pivot/SystemState", systemState.name());
    Logger.recordOutput("Subsystems/Pivot/WantedState", wantedState.name());
    Logger.recordOutput("Subsystems/Pivot/GoalAngle", goalAngle);
    Logger.recordOutput("Subsystems/Pivot/AtGoal", atGoal());
    Logger.recordOutput("Subsystems/Pivot/Angle", getAngle());
    Logger.recordOutput("Subsystems/Pivot/VelocityRadPerSec", getVelocity());
  }
}
