// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class RollerSystemIOSim implements RollerSystemIO {
  private final DCMotorSim sim;
  private final DCMotor gearbox;
  private double appliedVoltage = 0.0;

  public RollerSystemIOSim(DCMotor motorModel, double reduction, double moi) {
    gearbox = motorModel;
    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0.0);
    }

    sim.update(Constants.loopPeriodSecs);
    inputs.data =
        new RollerSystemIOData(
            sim.getAngularPositionRad(),
            sim.getAngularVelocityRadPerSec(),
            appliedVoltage,
            sim.getCurrentDrawAmps(),
            gearbox.getCurrent(sim.getAngularVelocityRadPerSec(), appliedVoltage),
            0.0,
            false,
            true);
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void runTorqueCurrent(double amps) {
    runVolts(gearbox.getVoltage(gearbox.getTorque(amps), sim.getAngularVelocityRadPerSec()));
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
