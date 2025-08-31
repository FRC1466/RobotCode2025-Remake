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
    public PivotIOData data =
        new PivotIOData(
            new Rotation2d(0),
            0,
            0,
            0,
            0,
            0,
            0);
  }

  record PivotIOData(
    Rotation2d angle,
    double appliedVolts,
    double supplyCurrentAmps,
    double statorCurrentAmps,
    double angularVelocityRotPerSec,
    double angularAccelerationRadPerSecSquared,
    double motorTemp) {}

  default void setTargetAngle(Rotation2d target) {}

  default void resetAngle(Rotation2d angle) {}

  default void setDutyCycle(double dutyCycle) {}

  default void setNeutralMode(NeutralModeValue neutralMode) {}

  default void setPID(double kP, double kI, double kD) {}
}
