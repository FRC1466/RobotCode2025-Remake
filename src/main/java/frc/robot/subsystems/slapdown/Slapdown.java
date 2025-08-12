// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.slapdown;

import static frc.robot.constants.SlapdownConstants.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SlapdownConstants;
import frc.robot.util.LoggedTracer;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/** The Slapdown subsystem controls the slapdown arm rotation using a simple state machine. */
public class Slapdown extends SubsystemBase {
  public enum WantedState {
    IDLE,
    MOVE_TO_POSITION
  }

  private enum SystemState {
    IDLING,
    MOVING_TO_POSITION
  }

  private final SlapdownIO io;
  private final SlapdownIOInputsAutoLogged inputs = new SlapdownIOInputsAutoLogged();

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  @Getter private Rotation2d goalAngle;

  private static final double kAngleToleranceRad = Units.degreesToRadians(2.0);

  public Slapdown(SlapdownIO io) {
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

  public void setNeutralMode(NeutralModeValue neutralModeValue) {
    io.setNeutralMode(neutralModeValue);
  }

  public Rotation2d getAngle() {
    return inputs.slapdownAngle;
  }

  public double getVelocity() {
    return inputs.slapdownAngularVelocityRadPerSec;
  }

  public double getAcceleration() {
    return inputs.slapdownAngularAccelerationRadPerSecSquared;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Subsystems/Slapdown", inputs);

    if (SlapdownConstants.kP.hasChanged(hashCode())
        || SlapdownConstants.kI.hasChanged(hashCode())
        || SlapdownConstants.kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get());
    }

    systemState = handleStateTransitions();
    applyStates();
    logState();
    LoggedTracer.record("Slapdown");
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

  private void logState() {
    Logger.recordOutput("Subsystems/Slapdown/SystemState", systemState.name());
    Logger.recordOutput("Subsystems/Slapdown/WantedState", wantedState.name());
    Logger.recordOutput("Subsystems/Slapdown/GoalAngle", goalAngle);
    Logger.recordOutput("Subsystems/Slapdown/AtGoal", atGoal());
    Logger.recordOutput("Subsystems/Slapdown/Angle", getAngle());
    Logger.recordOutput("Subsystems/Slapdown/VelocityRadPerSec", getVelocity());
  }
}
