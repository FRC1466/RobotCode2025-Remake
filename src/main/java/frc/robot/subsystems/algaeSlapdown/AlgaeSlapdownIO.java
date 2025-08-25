// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.algaeSlapdown;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeSlapdownIO {
  default void updateInputs(AlgaeSlapdownIOInputs inputs) {}

  @AutoLog
  class AlgaeSlapdownIOInputs {
    // Angle is relative to starting position
    public Rotation2d slapdownAngle = Rotation2d.kZero;

    public double slapdownAppliedVolts;
    public double slapdownSupplyCurrentAmps;
    public double slapdownStatorCurrentAmps;
    public double slapdownAngularVelocityRadPerSec;
    public double slapdownAngularAccelerationRadPerSecSquared;
    public double slapdownMotorTemp;
  }

  default void setTargetAngle(Rotation2d target) {}

  default void resetSlapdownAngle(Rotation2d angle) {}

  default void setDutyCycle(double dutyCycle) {}

  default void setNeutralMode(NeutralModeValue neutralMode) {}

  default void setPID(double kP, double kI, double kD) {}
}
