// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.diffwrist;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** IO layer for a differential wrist/pivot mechanism. */
public interface DifferentialWristPivotIO {
  default void updateInputs(DifferentialWristPivotIOInputs inputs) {}

  @AutoLog
  class DifferentialWristPivotIOInputs {
    // Mechanism angles relative to a defined zero
    public Rotation2d pivotAngle = Rotation2d.kZero;
    public Rotation2d wristAngle = Rotation2d.kZero;

    // Velocities and accelerations
    public double pivotAngularVelocityRadPerSec;
    public double pivotAngularAccelerationRadPerSecSquared;
    public double wristAngularVelocityRadPerSec;
    public double wristAngularAccelerationRadPerSecSquared;

    // Electrical telemetry
    public double pivotAppliedVolts;
    public double pivotSupplyCurrentAmps;
    public double pivotStatorCurrentAmps;
    public double wristAppliedVolts;
    public double wristSupplyCurrentAmps;
    public double wristStatorCurrentAmps;

    // Temperatures
    public double pivotMotorTempC;
    public double wristMotorTempC;
  }

  /** Command closed-loop angles for pivot and wrist. */
  default void setTargetAngles(Rotation2d pivotTarget, Rotation2d wristTarget) {}

  /** Reset measured mechanism angles. */
  default void resetAngles(Rotation2d pivotAngle, Rotation2d wristAngle) {}

  /** Open-loop duty for each axis in [-1, 1]. */
  default void setDutyCycles(double pivotDuty, double wristDuty) {}

  /** Set motor neutral behavior. */
  default void setNeutralMode(NeutralModeValue neutralMode) {}

  /** Configure PID for pivot axis. */
  default void setPIDPivot(double kP, double kI, double kD) {}

  /** Configure PID for wrist axis. */
  default void setPIDWrist(double kP, double kI, double kD) {}
}
