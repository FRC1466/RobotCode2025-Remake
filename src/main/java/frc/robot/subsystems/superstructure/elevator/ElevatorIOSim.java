// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.SuperstructurePose;

/**
 * This class is a simulation of the Elevator subsystem. It uses the WPILib ElevatorSim class to
 * simulate the physics of the elevator.
 */
public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim sim;
  private final PIDController pidController;
  private double appliedVolts = 0.0;
  private boolean brakeMode = true;

  public ElevatorIOSim() {
    sim =
        new ElevatorSim(
            DCMotor.getFalcon500(2),
            12,
            6,
            .0254,
            0.0,
            SuperstructureConstants.elevatorMaxTravel,
            true,
            0.0);
    pidController = new PIDController(0.0, 0.0, 0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.update(0.02);

    inputs.data =
        new ElevatorIOData(
            true,
            true,
            SuperstructurePose.metersToRotations(sim.getPositionMeters()),
            SuperstructurePose.metersToRotations(sim.getVelocityMetersPerSecond()),
            appliedVolts,
            sim.getCurrentDrawAmps(),
            sim.getCurrentDrawAmps(),
            0.0,
            appliedVolts,
            sim.getCurrentDrawAmps(),
            sim.getCurrentDrawAmps(),
            0.0);
  }

  @Override
  public void runOpenLoop(double output) {
    runVolts(output * 12.0);
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = volts;
    if (brakeMode && Math.abs(appliedVolts) < 1e-4) {
      sim.setInput(0.0);
    } else {
      sim.setInput(appliedVolts);
    }
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }

  @Override
  public void runPosition(double positionRad, double feedforward) {
    double setpointMeters = SuperstructurePose.rotationsToMeters(positionRad);
    double pidOutput = pidController.calculate(sim.getPositionMeters(), setpointMeters);
    runVolts(pidOutput + feedforward);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    pidController.setPID(kP, kI, kD);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    this.brakeMode = enabled;
  }
}
