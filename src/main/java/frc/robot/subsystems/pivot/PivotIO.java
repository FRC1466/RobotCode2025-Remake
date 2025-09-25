// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  default void updateInputs(PivotIOInputs inputs) {}

  @AutoLog
  class PivotIOInputs {
    // Angle is relative to starting position
    public Rotation2d pivotAngle = Rotation2d.kZero;

    public double pivotAppliedVolts;
    public double pivotSupplyCurrentAmps;
    public double pivotStatorCurrentAmps;
    public double pivotAngularVelocityRadPerSec;
    public double pivotAngularAccelerationRadPerSecSquared;
    public double pivotMotorTemp;
  }

  default void setTargetAngle(Rotation2d target) {}

  default void resetPivotAngle(Rotation2d angle) {}

  default void setDutyCycle(double dutyCycle) {}

  default void setNeutralMode(NeutralModeValue neutralMode) {}

  default void setPID(double kP, double kI, double kD) {}
}
