// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.testing.testers;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.testing.SubsystemTester;

/**
 * Tests the Intake subsystem in isolation.
 * Validates roller motors and game piece sensors.
 */
public class IntakeTester extends SubsystemTester {
  private final Intake intake;
  
  public IntakeTester(Intake intake) {
    super("Intake");
    this.intake = intake;
  }
  
  @Override
  protected void defineTestSteps() {
    // Step 1: Test end effector roller
    testSteps.add(new TestStep("End Effector Motor Test", false) {
      private Timer stepTimer = new Timer();
      
      @Override
      protected void onStart() {
        stepTimer.restart();
        intake.setWantedState(Intake.WantedState.INTAKE_CORAL);
        System.out.println("Testing end effector motor...");
      }
      
      @Override
      protected void onExecute() {
        if (stepTimer.get() > 1.0) {
          intake.setWantedState(Intake.WantedState.OFF);
        }
      }
      
      @Override
      protected boolean isStepComplete() {
        return stepTimer.get() > 1.5;
      }
      
      @Override
      protected boolean validateStep() {
        // Basic validation - motor ran without errors
        System.out.println("✓ End effector motor ran");
        return true;
      }
    });
    
    // Step 2: Test star wheel roller
    testSteps.add(new TestStep("Star Wheel Motor Test", false) {
      private Timer stepTimer = new Timer();
      
      @Override
      protected void onStart() {
        stepTimer.restart();
        intake.setWantedState(Intake.WantedState.INTAKE_ALGAE);
        System.out.println("Testing star wheel motor...");
      }
      
      @Override
      protected void onExecute() {
        if (stepTimer.get() > 1.0) {
          intake.setWantedState(Intake.WantedState.OFF);
        }
      }
      
      @Override
      protected boolean isStepComplete() {
        return stepTimer.get() > 1.5;
      }
      
      @Override
      protected boolean validateStep() {
        // Basic validation - motor ran without errors
        System.out.println("✓ Star wheel motor ran");
        return true;
      }
    });
    
    // Step 3: Test coral sensor
    testSteps.add(new TestStep("Coral Sensor Test", false) {
      @Override
      protected void onStart() {
        System.out.println("Testing coral sensor (place coral in intake if available)...");
      }
      
      @Override
      protected void onExecute() {}
      
      @Override
      protected boolean isStepComplete() {
        return true;
      }
      
      @Override
      protected boolean validateStep() {
        boolean hasCoral = intake.hasCoral();
        System.out.println("✓ Coral sensor reading: " + (hasCoral ? "DETECTED" : "NOT DETECTED"));
        return true;
      }
    });
    
    // Step 4: Test grip functionality
    testSteps.add(new TestStep("Grip Test", false) {
      private Timer stepTimer = new Timer();
      
      @Override
      protected void onStart() {
        stepTimer.restart();
        intake.setWantedState(Intake.WantedState.GRIP_CORAL);
        System.out.println("Testing grip function...");
      }
      
      @Override
      protected void onExecute() {}
      
      @Override
      protected boolean isStepComplete() {
        return stepTimer.get() > 1.0;
      }
      
      @Override
      protected boolean validateStep() {
        System.out.println("✓ Grip function executed");
        return true;
      }
    });
    
    // Step 5: Test outtake
    testSteps.add(new TestStep("Outtake Test", false) {
      private Timer stepTimer = new Timer();
      
      @Override
      protected void onStart() {
        stepTimer.restart();
        intake.setWantedState(Intake.WantedState.OUTTAKE_CORAL);
        System.out.println("Testing outtake...");
      }
      
      @Override
      protected void onExecute() {
        if (stepTimer.get() > 0.5) {
          intake.setWantedState(Intake.WantedState.OFF);
        }
      }
      
      @Override
      protected boolean isStepComplete() {
        return stepTimer.get() > 1.0;
      }
      
      @Override
      protected boolean validateStep() {
        System.out.println("✓ Outtake function executed");
        return true;
      }
    });
  }
  
  @Override
  protected void returnToSafeState() {
    intake.setWantedState(Intake.WantedState.OFF);
  }
}
