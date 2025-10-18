// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.testing;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/**
 * Abstract base class for subsystem testers.
 * Provides common functionality for running isolated subsystem tests.
 */
public abstract class SubsystemTester {
  
  public enum TestState {
    IDLE,
    RUNNING,
    PASSED,
    FAILED
  }
  
  protected final String subsystemName;
  @Getter protected TestState testState = TestState.IDLE;
  @Getter protected SubsystemHealthMonitor healthMonitor;
  
  protected final Timer testTimer = new Timer();
  protected int currentTestStep = 0;
  protected List<TestStep> testSteps = new ArrayList<>();
  
  public SubsystemTester(String subsystemName) {
    this.subsystemName = subsystemName;
    this.healthMonitor = new SubsystemHealthMonitor(subsystemName);
    defineTestSteps();
  }
  
  /**
   * Define the test steps for this subsystem.
   * Called during construction.
   */
  protected abstract void defineTestSteps();
  
  /**
   * Start the test sequence.
   */
  public void startTest() {
    testState = TestState.RUNNING;
    currentTestStep = 0;
    healthMonitor.clearChecks();
    testTimer.restart();
    
    Logger.recordOutput("Testing/" + subsystemName + "/TestState", testState.name());
    Logger.recordOutput("Testing/" + subsystemName + "/TestStartTime", Timer.getFPGATimestamp());
    
    System.out.println("=== Starting " + subsystemName + " Test ===");
  }
  
  /**
   * Stop the test and return subsystem to safe state.
   */
  public void stopTest() {
    testState = TestState.IDLE;
    currentTestStep = 0;
    returnToSafeState();
    
    Logger.recordOutput("Testing/" + subsystemName + "/TestState", testState.name());
    
    System.out.println("=== Stopped " + subsystemName + " Test ===");
    System.out.println(healthMonitor.getSummary());
  }
  
  /**
   * Called periodically while test is active.
   */
  public void periodic() {
    if (testState != TestState.RUNNING) {
      return;
    }
    
    // Check if current step is complete
    if (currentTestStep < testSteps.size()) {
      TestStep step = testSteps.get(currentTestStep);
      
      // Execute step if not started
      if (!step.isStarted()) {
        step.start();
      }
      
      // Run step logic
      step.execute();
      
      // Check if step is complete
      if (step.isComplete()) {
        // Validate step results
        boolean passed = step.validate();
        
        if (!passed && step.isCritical()) {
          // Critical step failed, stop test
          testState = TestState.FAILED;
          healthMonitor.addCheck(
              step.getName(), 
              false, 
              "CRITICAL FAILURE: " + step.getFailureReason());
          System.out.println("❌ Test FAILED at step: " + step.getName());
          System.out.println("   Reason: " + step.getFailureReason());
          returnToSafeState();
        } else {
          healthMonitor.addCheck(
              step.getName(), 
              passed, 
              passed ? "Step completed successfully" : step.getFailureReason());
          
          if (passed) {
            System.out.println("✅ Step " + (currentTestStep + 1) + "/" + testSteps.size() 
                + " PASSED: " + step.getName());
          } else {
            System.out.println("⚠️ Step " + (currentTestStep + 1) + "/" + testSteps.size() 
                + " WARNING: " + step.getName() + " - " + step.getFailureReason());
          }
        }
        
        currentTestStep++;
      }
    } else {
      // All steps complete
      testState = healthMonitor.getOverallHealth() == SubsystemHealthMonitor.HealthStatus.FAIL 
          ? TestState.FAILED : TestState.PASSED;
      
      System.out.println("=== " + subsystemName + " Test Complete ===");
      System.out.println("Result: " + testState);
      System.out.println(healthMonitor.getSummary());
      
      returnToSafeState();
    }
    
    // Log status
    Logger.recordOutput("Testing/" + subsystemName + "/TestState", testState.name());
    Logger.recordOutput("Testing/" + subsystemName + "/CurrentStep", currentTestStep);
    Logger.recordOutput("Testing/" + subsystemName + "/TotalSteps", testSteps.size());
    Logger.recordOutput("Testing/" + subsystemName + "/TestTime", testTimer.get());
    
    if (currentTestStep < testSteps.size()) {
      Logger.recordOutput("Testing/" + subsystemName + "/CurrentStepName", 
          testSteps.get(currentTestStep).getName());
    }
    
    healthMonitor.logHealth();
  }
  
  /**
   * Return the subsystem to a safe, idle state.
   */
  protected abstract void returnToSafeState();
  
  /**
   * Get progress percentage.
   */
  public double getProgress() {
    if (testSteps.isEmpty()) {
      return 0.0;
    }
    return (double) currentTestStep / testSteps.size() * 100.0;
  }
  
  /**
   * Abstract class representing a single test step.
   */
  protected abstract class TestStep {
    private final String name;
    private final boolean critical;
    private boolean started = false;
    private String failureReason = "";
    
    protected TestStep(String name, boolean critical) {
      this.name = name;
      this.critical = critical;
    }
    
    /**
     * Called when the step starts.
     */
    protected abstract void onStart();
    
    /**
     * Called every periodic cycle while step is running.
     */
    protected abstract void onExecute();
    
    /**
     * Check if this step is complete.
     */
    protected abstract boolean isStepComplete();
    
    /**
     * Validate the results of this step.
     * @return true if step passed, false otherwise
     */
    protected abstract boolean validateStep();
    
    public void start() {
      started = true;
      onStart();
      Logger.recordOutput("Testing/" + subsystemName + "/Step/" + name + "/Started", true);
    }
    
    public void execute() {
      onExecute();
    }
    
    public boolean isComplete() {
      return isStepComplete();
    }
    
    public boolean validate() {
      return validateStep();
    }
    
    public String getName() {
      return name;
    }
    
    public boolean isCritical() {
      return critical;
    }
    
    public boolean isStarted() {
      return started;
    }
    
    public void setFailureReason(String reason) {
      this.failureReason = reason;
    }
    
    public String getFailureReason() {
      return failureReason;
    }
  }
}
