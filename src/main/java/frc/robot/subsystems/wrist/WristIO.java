// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  default void updateInputs(WristIOInputs inputs) {}

  @AutoLog
  class WristIOInputs {
    // Angle is relative to starting position
    public Rotation2d wristAngle = Rotation2d.kZero;

    public double wristAppliedVolts;
    public double wristSupplyCurrentAmps;
    public double wristStatorCurrentAmps;
    public double wristAngularVelocityRadPerSec;
    public double wristAngularAccelerationRadPerSecSquared;
    public double wristMotorTemp;
  }

  default void setTargetAngle(Rotation2d target) {}

  default void resetWristAngle(Rotation2d angle) {}

  default void setDutyCycle(double dutyCycle) {}

  default void setNeutralMode(NeutralModeValue neutralMode) {}

  default void setPID(double kP, double kI, double kD) {}
}
