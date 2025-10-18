// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.testing;

import java.util.ArrayList;
import java.util.List;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks the health and status of a subsystem during testing.
 * Provides diagnostic information to identify sim-to-real issues.
 */
public class SubsystemHealthMonitor {
  
  public enum HealthStatus {
    UNKNOWN("Unknown", "⚪"),
    PASS("Pass", "✅"),
    WARNING("Warning", "⚠️"),
    FAIL("Fail", "❌");
    
    private final String name;
    private final String icon;
    
    HealthStatus(String name, String icon) {
      this.name = name;
      this.icon = icon;
    }
    
    public String getName() {
      return name;
    }
    
    public String getIcon() {
      return icon;
    }
    
    @Override
    public String toString() {
      return icon + " " + name;
    }
  }
  
  private final String subsystemName;
  @Getter private HealthStatus overallHealth = HealthStatus.UNKNOWN;
  
  private final List<HealthCheck> checks = new ArrayList<>();
  
  public SubsystemHealthMonitor(String subsystemName) {
    this.subsystemName = subsystemName;
  }
  
  /**
   * Add a health check to monitor.
   */
  public void addCheck(String name, boolean passed, String details) {
    checks.add(new HealthCheck(name, passed ? HealthStatus.PASS : HealthStatus.FAIL, details));
    updateOverallHealth();
  }
  
  /**
   * Add a warning check.
   */
  public void addWarning(String name, String details) {
    checks.add(new HealthCheck(name, HealthStatus.WARNING, details));
    updateOverallHealth();
  }
  
  /**
   * Clear all checks.
   */
  public void clearChecks() {
    checks.clear();
    overallHealth = HealthStatus.UNKNOWN;
  }
  
  /**
   * Update the overall health based on all checks.
   */
  private void updateOverallHealth() {
    if (checks.isEmpty()) {
      overallHealth = HealthStatus.UNKNOWN;
      return;
    }
    
    boolean hasFail = checks.stream().anyMatch(c -> c.status == HealthStatus.FAIL);
    boolean hasWarning = checks.stream().anyMatch(c -> c.status == HealthStatus.WARNING);
    
    if (hasFail) {
      overallHealth = HealthStatus.FAIL;
    } else if (hasWarning) {
      overallHealth = HealthStatus.WARNING;
    } else {
      overallHealth = HealthStatus.PASS;
    }
  }
  
  /**
   * Log health status to AdvantageKit.
   */
  public void logHealth() {
    String prefix = "Testing/" + subsystemName + "/Health/";
    
    Logger.recordOutput(prefix + "OverallStatus", overallHealth.toString());
    
    for (int i = 0; i < checks.size(); i++) {
      HealthCheck check = checks.get(i);
      Logger.recordOutput(prefix + "Check_" + i + "_Name", check.name);
      Logger.recordOutput(prefix + "Check_" + i + "_Status", check.status.toString());
      Logger.recordOutput(prefix + "Check_" + i + "_Details", check.details);
    }
    
    Logger.recordOutput(prefix + "TotalChecks", checks.size());
  }
  
  /**
   * Get a summary string of all checks.
   */
  public String getSummary() {
    StringBuilder sb = new StringBuilder();
    sb.append(subsystemName).append(" Health: ").append(overallHealth).append("\n");
    
    for (HealthCheck check : checks) {
      sb.append("  ").append(check.status.getIcon()).append(" ")
        .append(check.name).append(": ").append(check.details).append("\n");
    }
    
    return sb.toString();
  }
  
  /**
   * Get all checks.
   */
  public List<HealthCheck> getChecks() {
    return new ArrayList<>(checks);
  }
  
  /**
   * Internal class representing a single health check.
   */
  public static class HealthCheck {
    public final String name;
    public final HealthStatus status;
    public final String details;
    
    public HealthCheck(String name, HealthStatus status, String details) {
      this.name = name;
      this.status = status;
      this.details = details;
    }
  }
}
