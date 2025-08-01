// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.archive.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSystemIO {
  @AutoLog
  class RollerSystemIOInputs {
    public RollerSystemIOData data = new RollerSystemIOData(0, 0, 0, 0, 0, 0, false, false);
  }

  record RollerSystemIOData(
      double positionRads,
      double velocityRadsPerSec,
      double appliedVoltage,
      double supplyCurrentAmps,
      double torqueCurrentAmps,
      double tempCelsius,
      boolean tempFault,
      boolean connected) {}

  default void updateInputs(RollerSystemIOInputs inputs) {}

  /* Run rollers at volts */
  default void runVolts(double volts) {}

  default void runTorqueCurrent(double amps) {}

  /* Stop rollers */
  default void stop() {}

  default void setCurrentLimit(double currentLimit) {}

  default void setBrakeMode(boolean enabled) {}
}
