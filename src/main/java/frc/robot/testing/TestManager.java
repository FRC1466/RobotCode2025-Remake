// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.testing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Central manager for subsystem testing. Coordinates test execution,
 * subsystem isolation, and result reporting.
 */
public class TestManager extends SubsystemBase {
  @Getter private SubsystemTestMode currentTestMode = SubsystemTestMode.NONE;
  @Getter private boolean testModeActive = false;
  
  private final Map<SubsystemTestMode, SubsystemTester> testers = new HashMap<>();
  private SubsystemTester activeTest = null;
  
  private final LoggedDashboardChooser<SubsystemTestMode> testModeChooser =
      new LoggedDashboardChooser<>("Subsystem Test Mode");
  
  // Callback to disable choreographer during tests
  private Consumer<Boolean> choreographerDisableCallback = null;
  
  public TestManager() {
    // Setup test mode chooser
    testModeChooser.addDefaultOption(SubsystemTestMode.NONE.getName(), SubsystemTestMode.NONE);
    testModeChooser.addOption(SubsystemTestMode.DRIVE.getName(), SubsystemTestMode.DRIVE);
    testModeChooser.addOption(SubsystemTestMode.ELEVATOR.getName(), SubsystemTestMode.ELEVATOR);
    testModeChooser.addOption(SubsystemTestMode.PIVOT.getName(), SubsystemTestMode.PIVOT);
    testModeChooser.addOption(SubsystemTestMode.INTAKE.getName(), SubsystemTestMode.INTAKE);
    testModeChooser.addOption(SubsystemTestMode.VISION.getName(), SubsystemTestMode.VISION);
    testModeChooser.addOption(SubsystemTestMode.ALL.getName(), SubsystemTestMode.ALL);
  }
  
  /**
   * Set callback to disable/enable choreographer during tests.
   * @param callback Function that takes boolean (true = disable, false = enable)
   */
  public void setChoreographerDisableCallback(Consumer<Boolean> callback) {
    this.choreographerDisableCallback = callback;
  }
  
  /**
   * Register a subsystem tester for a specific test mode.
   * @param mode The test mode this tester handles
   * @param tester The tester implementation
   */
  public void registerTester(SubsystemTestMode mode, SubsystemTester tester) {
    testers.put(mode, tester);
  }
  
  /**
   * Check if a subsystem should be active based on current test mode.
   * @param subsystemName Name of the subsystem to check
   * @return true if subsystem should be active, false if it should be disabled
   */
  public boolean isSubsystemActive(String subsystemName) {
    if (!testModeActive || currentTestMode == SubsystemTestMode.NONE) {
      return true; // All subsystems active in normal mode
    }
    
    if (currentTestMode == SubsystemTestMode.ALL) {
      return true; // All subsystems active when testing all
    }
    
    // Only activate the subsystem being tested
    return switch (currentTestMode) {
      case DRIVE -> subsystemName.equalsIgnoreCase("Drive");
      case ELEVATOR -> subsystemName.equalsIgnoreCase("Elevator");
      case PIVOT -> subsystemName.equalsIgnoreCase("Pivot") || subsystemName.equalsIgnoreCase("Wrist");
      case INTAKE -> subsystemName.equalsIgnoreCase("Intake");
      case VISION -> subsystemName.equalsIgnoreCase("Vision");
      default -> false;
    };
  }
  
  /**
   * Start a test for the specified mode.
   * @param mode The test mode to run
   * @return Command that runs the test
   */
  public Command startTest(SubsystemTestMode mode) {
    return Commands.runOnce(() -> {
      if (mode == SubsystemTestMode.NONE) {
        stopTest();
        return;
      }
      
      currentTestMode = mode;
      testModeActive = true;
      activeTest = testers.get(mode);
      
      // Disable choreographer to prevent interference
      if (choreographerDisableCallback != null) {
        choreographerDisableCallback.accept(true);
        System.out.println("⚠️ CHOREOGRAPHER DISABLED FOR TESTING");
      }
      
      if (activeTest != null) {
        activeTest.startTest();
      }
      
      Logger.recordOutput("Testing/TestMode", currentTestMode.getName());
      Logger.recordOutput("Testing/TestActive", testModeActive);
    }).withName("Start Test: " + mode.getName());
  }
  
  /**
   * Start test based on dashboard chooser selection.
   */
  public Command startSelectedTest() {
    return Commands.defer(() -> startTest(testModeChooser.get()), java.util.Set.of(this));
  }
  
  /**
   * Stop the current test and return to normal operation.
   */
  public void stopTest() {
    if (activeTest != null) {
      activeTest.stopTest();
      activeTest = null;
    }
    
    testModeActive = false;
    currentTestMode = SubsystemTestMode.NONE;
    
    // Re-enable choreographer
    if (choreographerDisableCallback != null) {
      choreographerDisableCallback.accept(false);
      System.out.println("✅ CHOREOGRAPHER RE-ENABLED");
    }
    
    Logger.recordOutput("Testing/TestMode", "None");
    Logger.recordOutput("Testing/TestActive", false);
  }
  
  /**
   * Command to stop the current test.
   */
  public Command stopTestCommand() {
    return Commands.runOnce(this::stopTest).withName("Stop Test");
  }
  
  @Override
  public void periodic() {
    // Update active test
    if (activeTest != null) {
      activeTest.periodic();
    }
    
    // Log test status
    Logger.recordOutput("Testing/TestMode", currentTestMode.getName());
    Logger.recordOutput("Testing/TestActive", testModeActive);
    
    SmartDashboard.putString("Test Mode", currentTestMode.getName());
    SmartDashboard.putBoolean("Test Active", testModeActive);
  }
  
  /**
   * Get the active tester if one is running.
   */
  public SubsystemTester getActiveTester() {
    return activeTest;
  }
}
