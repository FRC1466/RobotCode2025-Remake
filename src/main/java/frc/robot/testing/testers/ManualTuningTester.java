// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.testing.testers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.testing.SubsystemTester;

/**
 * Manual tuning mode for testing and creating setpoints. Exposes NetworkTables fields to control
 * subsystem positions directly. Ideal for simulation and real robot setpoint development.
 */
public class ManualTuningTester extends SubsystemTester {
  private final Elevator elevator;
  private final Pivot pivot;

  // NetworkTables keys
  private static final String NT_ELEVATOR_SETPOINT_INCHES = "Manual/ElevatorSetpoint_Inches";
  private static final String NT_ELEVATOR_CURRENT_INCHES = "Manual/ElevatorCurrent_Inches";
  private static final String NT_ELEVATOR_AT_GOAL = "Manual/ElevatorAtGoal";
  private static final String NT_ELEVATOR_ENABLED = "Manual/ElevatorEnabled";

  private static final String NT_PIVOT_SETPOINT_DEGREES = "Manual/PivotSetpoint_Degrees";
  private static final String NT_PIVOT_CURRENT_DEGREES = "Manual/PivotCurrent_Degrees";
  private static final String NT_PIVOT_AT_GOAL = "Manual/PivotAtGoal";
  private static final String NT_PIVOT_ENABLED = "Manual/PivotEnabled";

  private static final String NT_INSTRUCTIONS = "Manual/Instructions";

  public ManualTuningTester(Elevator elevator, Pivot pivot) {
    super("Manual Tuning");
    this.elevator = elevator;
    this.pivot = pivot;
  }

  @Override
  protected void defineTestSteps() {
    // Manual mode has one continuous step
    testSteps.add(
        new TestStep("Manual Control Active", false) {
          private Timer stepTimer = new Timer();

          @Override
          protected void onStart() {
            stepTimer.restart();
            System.out.println("=== Manual Tuning Mode Active ===");
            System.out.println("Use NetworkTables to control subsystem positions:");
            System.out.println("  - " + NT_ELEVATOR_SETPOINT_INCHES);
            System.out.println("  - " + NT_PIVOT_SETPOINT_DEGREES);
            System.out.println("  - " + NT_ELEVATOR_ENABLED + " / " + NT_PIVOT_ENABLED);
            System.out.println("Press B+Start to exit.");

            // Initialize NetworkTables with current positions
            SmartDashboard.putNumber(
                NT_ELEVATOR_SETPOINT_INCHES, Units.metersToInches(elevator.getPosition()));
            SmartDashboard.putNumber(NT_PIVOT_SETPOINT_DEGREES, pivot.getAngle().getDegrees());
            SmartDashboard.putBoolean(NT_ELEVATOR_ENABLED, true);
            SmartDashboard.putBoolean(NT_PIVOT_ENABLED, true);

            SmartDashboard.putString(
                NT_INSTRUCTIONS,
                "Manual Tuning Mode:\n"
                    + "1. Enable/disable subsystems using toggles\n"
                    + "2. Set target positions in NetworkTables\n"
                    + "3. Watch subsystems move to setpoints\n"
                    + "4. Record good setpoints for later use\n"
                    + "5. Press B+Start to exit");
          }

          @Override
          protected void onExecute() {
            // Read enable states
            boolean elevatorEnabled = SmartDashboard.getBoolean(NT_ELEVATOR_ENABLED, true);
            boolean pivotEnabled = SmartDashboard.getBoolean(NT_PIVOT_ENABLED, true);

            // Control elevator if enabled
            if (elevatorEnabled) {
              double targetInches =
                  SmartDashboard.getNumber(
                      NT_ELEVATOR_SETPOINT_INCHES, Units.metersToInches(elevator.getPosition()));
              double targetMeters = Units.inchesToMeters(targetInches);

              // Clamp to safe range (0 to 36 inches)
              targetMeters = Math.max(0.0, Math.min(Units.inchesToMeters(36.0), targetMeters));

              elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, targetMeters);

              // Publish current state
              SmartDashboard.putNumber(
                  NT_ELEVATOR_CURRENT_INCHES, Units.metersToInches(elevator.getPosition()));
              SmartDashboard.putBoolean(
                  NT_ELEVATOR_AT_GOAL, elevator.atGoal(Units.inchesToMeters(0.5)));
            } else {
              elevator.setWantedState(Elevator.WantedState.IDLE);
              SmartDashboard.putNumber(
                  NT_ELEVATOR_CURRENT_INCHES, Units.metersToInches(elevator.getPosition()));
            }

            // Control pivot if enabled
            if (pivotEnabled) {
              double targetDegrees =
                  SmartDashboard.getNumber(
                      NT_PIVOT_SETPOINT_DEGREES, pivot.getAngle().getDegrees());

              // Clamp to safe range (-10 to 120 degrees)
              targetDegrees = Math.max(-10.0, Math.min(120.0, targetDegrees));

              Rotation2d targetAngle = Rotation2d.fromDegrees(targetDegrees);
              pivot.setWantedState(Pivot.WantedState.MOVE_TO_POSITION, targetAngle);

              // Publish current state
              SmartDashboard.putNumber(NT_PIVOT_CURRENT_DEGREES, pivot.getAngle().getDegrees());
              SmartDashboard.putBoolean(NT_PIVOT_AT_GOAL, pivot.atGoal());
            } else {
              pivot.setWantedState(Pivot.WantedState.IDLE);
              SmartDashboard.putNumber(NT_PIVOT_CURRENT_DEGREES, pivot.getAngle().getDegrees());
            }

            // Log positions periodically
            if (stepTimer.hasElapsed(1.0)) {
              System.out.println(
                  String.format(
                      "Elevator: %.2f in (target: %.2f in, at goal: %b) | Pivot: %.1f° (target: %.1f°, at goal: %b)",
                      Units.metersToInches(elevator.getPosition()),
                      SmartDashboard.getNumber(NT_ELEVATOR_SETPOINT_INCHES, 0.0),
                      elevator.atGoal(Units.inchesToMeters(0.5)),
                      pivot.getAngle().getDegrees(),
                      SmartDashboard.getNumber(NT_PIVOT_SETPOINT_DEGREES, 0.0),
                      pivot.atGoal()));
              stepTimer.restart();
            }
          }

          @Override
          protected boolean isStepComplete() {
            // Manual mode never completes automatically - user must stop it
            return false;
          }

          @Override
          protected boolean validateStep() {
            // Always passes - this is manual control
            return true;
          }
        });
  }

  @Override
  protected void returnToSafeState() {
    elevator.setWantedState(Elevator.WantedState.IDLE);
    pivot.setWantedState(Pivot.WantedState.IDLE);

    System.out.println("=== Manual Tuning Mode Stopped ===");
    System.out.println("Subsystems returned to IDLE state.");

    // Clear NetworkTables
    SmartDashboard.putString(
        NT_INSTRUCTIONS, "Manual mode stopped. Select and start to re-enable.");
  }
}
