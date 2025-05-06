// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.util.LoggedTracer;
import java.util.function.DoubleSupplier;

public class Superstructure extends SubsystemBase {
  private final Elevator elevator;

  private final SuperstructureVisualizer measuredVisualizer =
      new SuperstructureVisualizer("Measured");
  private final SuperstructureVisualizer setpointVisualizer =
      new SuperstructureVisualizer("Setpoint");
  private final SuperstructureVisualizer goalVisualizer = new SuperstructureVisualizer("Goal");

  public Superstructure(Elevator elevator) {
    this.elevator = elevator;
  }

  @Override
  public void periodic() {
    // Run periodic
    elevator.periodic();

    // Update visualizer
    measuredVisualizer.update(elevator.getPositionMeters());
    setpointVisualizer.update(elevator.getSetpoint().position);
    goalVisualizer.update(elevator.getGoalMeters());

    // Record cycle time
    LoggedTracer.record("Superstructure");
  }

  private Command runElevator(DoubleSupplier elevatorHeight) {
    return Commands.runOnce(() -> elevator.setGoal(elevatorHeight));
  }
}
