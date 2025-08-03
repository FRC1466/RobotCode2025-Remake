// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * Simulated implementation of the ElevatorIO interface for development and testing when not at the lab.
 * This class simulates the elevator's behavior using the ElevatorSim class from WPILib.
 * To learn more on implementing WPILib simulations, view their documentation here:
 * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html
 */
public class ElevatorIOSim implements ElevatorIO {
  private static final double gearRatio = 12.0;
  private static final double drumRadius = 0.0254;
  private static final double minHeight = 0.0;
  private static final double maxHeight = 2.0;
  private static final double massKg = 7.0;

  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getKrakenX60(2), gearRatio, massKg, drumRadius, minHeight, maxHeight, true, 0.0);

  private final PIDController pid = new PIDController(5.0, 0.0, 0.0);
  private double appliedVoltage = 0.0;
  private boolean closedLoop = false;
  private double targetPosition = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.setInputVoltage(appliedVoltage);
    sim.update(0.02);

    double prevVelocity = inputs.elevatorVelocityMetersPerSec;

    inputs.elevatorPositionMeters = sim.getPositionMeters();
    inputs.elevatorVelocityMetersPerSec = sim.getVelocityMetersPerSecond();
    inputs.elevatorAccelerationMetersPerSecSquared =
        (inputs.elevatorVelocityMetersPerSec - prevVelocity) / 0.02;

    inputs.elevatorAppliedVolts = appliedVoltage;
    inputs.elevatorSupplyCurrentAmps = Math.abs(appliedVoltage) * 10.0;
    inputs.elevatorStatorCurrentAmps = inputs.elevatorSupplyCurrentAmps;
    inputs.elevatorMasterMotorTemp = 40.0;
    inputs.elevatorFollowerMotorTemp = 40.0;

    if (closedLoop) {
      appliedVoltage = pid.calculate(sim.getPositionMeters(), targetPosition);
      appliedVoltage = Math.max(-12.0, Math.min(12.0, appliedVoltage));
    }
  }

  @Override
  public void setTargetPosition(double positionMeters) {
    closedLoop = true;
    targetPosition = Math.max(minHeight, Math.min(maxHeight, positionMeters));
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    closedLoop = false;
    appliedVoltage = dutyCycle * RobotController.getBatteryVoltage();
  }

  @Override
  public void resetElevatorPosition(double positionMeters) {
    sim.setState(positionMeters, 0.0);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
