// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.testing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

/**
 * Dashboard interface for displaying test results and controlling tests.
 * Publishes data to NetworkTables for Shuffleboard/AdvantageScope visualization.
 */
public class TestDashboard {
  private final TestManager testManager;
  
  public TestDashboard(TestManager testManager) {
    this.testManager = testManager;
  }
  
  /**
   * Update all dashboard outputs.
   * Call this periodically (in Robot.robotPeriodic() or similar).
   */
  public void updateDashboard() {
    // Basic test status
    SmartDashboard.putString("Test/CurrentMode", testManager.getCurrentTestMode().getName());
    SmartDashboard.putBoolean("Test/Active", testManager.isTestModeActive());
    
    // Log to AdvantageKit
    Logger.recordOutput("Test/CurrentMode", testManager.getCurrentTestMode().getName());
    Logger.recordOutput("Test/Active", testManager.isTestModeActive());
    
    // If test is running, show details
    SubsystemTester activeTester = testManager.getActiveTester();
    if (activeTester != null) {
      SmartDashboard.putString("Test/State", activeTester.getTestState().name());
      SmartDashboard.putNumber("Test/Progress", activeTester.getProgress());
      SmartDashboard.putString("Test/Health", 
          activeTester.getHealthMonitor().getOverallHealth().toString());
      
      Logger.recordOutput("Test/State", activeTester.getTestState().name());
      Logger.recordOutput("Test/Progress", activeTester.getProgress());
      Logger.recordOutput("Test/HealthStatus", 
          activeTester.getHealthMonitor().getOverallHealth().toString());
      
      // Show health checks
      var checks = activeTester.getHealthMonitor().getChecks();
      SmartDashboard.putNumber("Test/TotalChecks", checks.size());
      
      int passCount = 0;
      int failCount = 0;
      int warnCount = 0;
      
      for (var check : checks) {
        switch (check.status) {
          case PASS -> passCount++;
          case FAIL -> failCount++;
          case WARNING -> warnCount++;
          default -> {}
        }
      }
      
      SmartDashboard.putNumber("Test/Passed", passCount);
      SmartDashboard.putNumber("Test/Failed", failCount);
      SmartDashboard.putNumber("Test/Warnings", warnCount);
      
      Logger.recordOutput("Test/PassedChecks", passCount);
      Logger.recordOutput("Test/FailedChecks", failCount);
      Logger.recordOutput("Test/WarningChecks", warnCount);
      
      // Create summary string
      StringBuilder summary = new StringBuilder();
      summary.append(String.format("Progress: %.0f%%\n", activeTester.getProgress()));
      summary.append(String.format("Checks: %d Pass, %d Fail, %d Warn\n", 
          passCount, failCount, warnCount));
      summary.append("Overall: ").append(
          activeTester.getHealthMonitor().getOverallHealth().toString());
      
      SmartDashboard.putString("Test/Summary", summary.toString());
    } else {
      // Clear test details when no test is running
      SmartDashboard.putString("Test/State", "IDLE");
      SmartDashboard.putNumber("Test/Progress", 0.0);
      SmartDashboard.putString("Test/Summary", "No test running");
    }
  }
  
  /**
   * Create dashboard instructions for using the test system.
   */
  public void publishInstructions() {
    String instructions = """
        === SUBSYSTEM TESTING SYSTEM ===
        
        1. SELECT TEST MODE from chooser
        2. Press START TEST button
        3. Watch test progress and health status
        4. Review results in AdvantageScope
        
        IMPORTANT:
        - Tests run ONLY the selected subsystem
        - Keep robot on blocks for safety
        - Some tests require manual movement
        - Check DriverStation console for details
        
        TEST MODES:
        - NONE: Normal operation
        - DRIVE: Test odometry, gyro, movement
        - ELEVATOR: Test position control, sensors
        - PIVOT: Test angle control
        - INTAKE: Test rollers and sensors
        - ALL: Run all tests in sequence
        """;
    
    SmartDashboard.putString("Test/Instructions", instructions);
  }
}
