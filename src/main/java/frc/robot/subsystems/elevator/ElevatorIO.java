// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  default void updateInputs(ExtensionIOInputs inputs) {}

  @AutoLog
  class ExtensionIOInputs {
    public double extensionPositionInMeters;

    public double extensionAppliedVolts;
    public double extensionSupplyCurrentAmps;
    public double extensionStatorCurrentAmps;
    public double extensionVelocityMetersPerSec;
    public double extensionAccelerationMetersPerSecSquared;

    public double extensionOneMotorTemp;
    public double extensionTwoMotorTemp;
    public double extensionThreeMotorTemp;
  }

  default void setShoulderAngleSupplier(Supplier<Rotation2d> shoulderAngleSupplier) {}

  default void setTargetExtension(double positionInMeters) {}

  default void resetExtensionPosition(double positionInMeters) {}

  default void setDutyCycle(double dutyCycle) {}

  default void setNeutralMode(NeutralModeValue neutralMode) {}
}
