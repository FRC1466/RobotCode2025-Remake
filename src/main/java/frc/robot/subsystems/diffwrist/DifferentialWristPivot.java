// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.diffwrist;

import static frc.robot.constants.DifferentialWristPivotConstants.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for a differential wrist + pivot mechanism, with simple 2-axis control and logging.
 * This mirrors the style of the other subsystems and relies on an IO layer for hardware.
 */
public class DifferentialWristPivot extends SubsystemBase {
  public enum WantedState {
    IDLE,
    MOVE_TO_POSITIONS
  }

  private enum SystemState {
    IDLING,
    MOVING
  }

  private final DifferentialWristPivotIO io;
  private final DifferentialWristPivotIOInputsAutoLogged inputs =
      new DifferentialWristPivotIOInputsAutoLogged();

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  private Rotation2d goalPivot = defaultPivotAngle;
  private Rotation2d goalWrist = defaultWristAngle;

  public DifferentialWristPivot(DifferentialWristPivotIO io) {
    this.io = io;
  }

  public void setWantedState(WantedState state) {
    this.wantedState = state;
  }

  public void setWantedState(WantedState state, Rotation2d pivot, Rotation2d wrist) {
    this.wantedState = state;
    setGoals(pivot, wrist);
  }

  public void setGoals(Rotation2d pivot, Rotation2d wrist) {
    this.goalPivot = clampPivot(pivot);
    this.goalWrist = clampWrist(wrist);
  }

  public Rotation2d getPivotAngle() {
    return inputs.pivotAngle;
  }

  public Rotation2d getWristAngle() {
    return inputs.wristAngle;
  }

  public boolean atGoal() {
    double tol = Units.degreesToRadians(3.0);
    return MathUtil.isNear(inputs.pivotAngle.getRadians(), goalPivot.getRadians(), tol)
        && MathUtil.isNear(inputs.wristAngle.getRadians(), goalWrist.getRadians(), tol);
  }

  public void setNeutralMode(NeutralModeValue neutralModeValue) {
    io.setNeutralMode(neutralModeValue);
  }

  private Rotation2d clampPivot(Rotation2d r) {
    return Rotation2d.fromRadians(MathUtil.clamp(r.getRadians(), pivotMinRadians, pivotMaxRadians));
  }

  private Rotation2d clampWrist(Rotation2d r) {
    return Rotation2d.fromRadians(MathUtil.clamp(r.getRadians(), wristMinRadians, wristMaxRadians));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Subsystems/DiffWristPivot", inputs);

    // PID retuning on-the-fly
    if (kP_Pivot.hasChanged(hashCode())
        || kI_Pivot.hasChanged(hashCode())
        || kD_Pivot.hasChanged(hashCode())) {
      io.setPIDPivot(kP_Pivot.get(), kI_Pivot.get(), kD_Pivot.get());
    }
    if (kP_Wrist.hasChanged(hashCode())
        || kI_Wrist.hasChanged(hashCode())
        || kD_Wrist.hasChanged(hashCode())) {
      io.setPIDWrist(kP_Wrist.get(), kI_Wrist.get(), kD_Wrist.get());
    }

    // State machine
    systemState =
        switch (wantedState) {
          case IDLE -> SystemState.IDLING;
          case MOVE_TO_POSITIONS -> SystemState.MOVING;
        };

    switch (systemState) {
      case IDLING -> io.setDutyCycles(0.0, 0.0);
      case MOVING -> io.setTargetAngles(goalPivot, goalWrist);
    }

    // Logging
    Logger.recordOutput("Subsystems/DiffWristPivot/SystemState", systemState.name());
    Logger.recordOutput("Subsystems/DiffWristPivot/WantedState", wantedState.name());
    Logger.recordOutput("Subsystems/DiffWristPivot/GoalPivot", goalPivot);
    Logger.recordOutput("Subsystems/DiffWristPivot/GoalWrist", goalWrist);
    Logger.recordOutput("Subsystems/DiffWristPivot/AtGoal", atGoal());
  }
}
