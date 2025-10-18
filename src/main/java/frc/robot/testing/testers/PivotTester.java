// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.testing.testers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.testing.SubsystemTester;

/**
 * Tests the Pivot/Wrist subsystem in isolation.
 * Validates motor function, angle control, and sensor readings.
 */
public class PivotTester extends SubsystemTester {
  private final Pivot pivot;
  
  private static final double TEST_TIMEOUT = 5.0;
  
  public PivotTester(Pivot pivot) {
    super("Pivot");
    this.pivot = pivot;
  }
  
  @Override
  protected void defineTestSteps() {
    // Step 1: Sensor connectivity check
    testSteps.add(new TestStep("Sensor Connectivity", true) {
      @Override
      protected void onStart() {
        System.out.println("Checking pivot angle sensor...");
      }
      
      @Override
      protected void onExecute() {}
      
      @Override
      protected boolean isStepComplete() {
        return true;
      }
      
      @Override
      protected boolean validateStep() {
        Rotation2d angle = pivot.getAngle();
        
        if (Double.isNaN(angle.getRadians()) || Double.isInfinite(angle.getRadians())) {
          setFailureReason("Angle sensor returning invalid data");
          return false;
        }
        
        System.out.println("✓ Angle sensor connected. Current angle: " + 
            String.format("%.1f", angle.getDegrees()) + "°");
        return true;
      }
    });
    
    // Step 2: Move to test position 1
    testSteps.add(new TestStep("Position Control Test 1", false) {
      private Timer stepTimer = new Timer();
      private Rotation2d targetAngle;
      
      @Override
      protected void onStart() {
        stepTimer.restart();
        targetAngle = Rotation2d.fromDegrees(45.0);
        pivot.setWantedState(Pivot.WantedState.MOVE_TO_POSITION, targetAngle);
        System.out.println("Moving to 45°...");
      }
      
      @Override
      protected void onExecute() {}
      
      @Override
      protected boolean isStepComplete() {
        return pivot.atGoal() || stepTimer.hasElapsed(TEST_TIMEOUT);
      }
      
      @Override
      protected boolean validateStep() {
        boolean atGoal = pivot.atGoal();
        
        if (!atGoal) {
          setFailureReason("Failed to reach 45°. Current: " + 
              String.format("%.1f", pivot.getAngle().getDegrees()) + "°");
          return false;
        }
        
        System.out.println("✓ Reached 45° in " + String.format("%.2f", stepTimer.get()) + "s");
        return true;
      }
    });
    
    // Step 3: Move to different position
    testSteps.add(new TestStep("Position Control Test 2", false) {
      private Timer stepTimer = new Timer();
      private Rotation2d targetAngle;
      
      @Override
      protected void onStart() {
        stepTimer.restart();
        targetAngle = Rotation2d.fromDegrees(90.0);
        pivot.setWantedState(Pivot.WantedState.MOVE_TO_POSITION, targetAngle);
        System.out.println("Moving to 90°...");
      }
      
      @Override
      protected void onExecute() {}
      
      @Override
      protected boolean isStepComplete() {
        return pivot.atGoal() || stepTimer.hasElapsed(TEST_TIMEOUT);
      }
      
      @Override
      protected boolean validateStep() {
        boolean atGoal = pivot.atGoal();
        
        if (!atGoal) {
          setFailureReason("Failed to reach 90°. Current: " + 
              String.format("%.1f", pivot.getAngle().getDegrees()) + "°");
          return false;
        }
        
        System.out.println("✓ Reached 90° in " + String.format("%.2f", stepTimer.get()) + "s");
        return true;
      }
    });
    
    // Step 4: Return to home
    testSteps.add(new TestStep("Return to Home", false) {
      private Timer stepTimer = new Timer();
      
      @Override
      protected void onStart() {
        stepTimer.restart();
        pivot.setWantedState(Pivot.WantedState.MOVE_TO_POSITION, pivot.getGoalAngle());
        System.out.println("Returning to home...");
      }
      
      @Override
      protected void onExecute() {}
      
      @Override
      protected boolean isStepComplete() {
        return pivot.atGoal() || stepTimer.hasElapsed(TEST_TIMEOUT);
      }
      
      @Override
      protected boolean validateStep() {
        if (!pivot.atGoal()) {
          setFailureReason("Failed to return home");
          return false;
        }
        
        System.out.println("✓ Returned to home");
        return true;
      }
    });
  }
  
  @Override
  protected void returnToSafeState() {
    pivot.setWantedState(Pivot.WantedState.IDLE);
  }
}
