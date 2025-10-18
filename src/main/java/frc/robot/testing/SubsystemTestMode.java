// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.testing;

/**
 * Enum representing the different subsystem test modes available.
 * These modes allow isolated testing of individual subsystems.
 */
public enum SubsystemTestMode {
  /** No testing, normal operation */
  NONE("None"),
  
  /** Test only the drive subsystem */
  DRIVE("Drive"),
  
  /** Test only the elevator subsystem */
  ELEVATOR("Elevator"),
  
  /** Test only the pivot/wrist subsystem */
  PIVOT("Pivot"),
  
  /** Test only the intake subsystem */
  INTAKE("Intake"),
  
  /** Test vision subsystem */
  VISION("Vision"),
  
  /** Run all subsystem tests in sequence */
  ALL("All Subsystems");
  
  private final String name;
  
  SubsystemTestMode(String name) {
    this.name = name;
  }
  
  public String getName() {
    return name;
  }
  
  @Override
  public String toString() {
    return name;
  }
}
