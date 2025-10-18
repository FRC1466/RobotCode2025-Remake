// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

public class CanDevice {
  private final int deviceNumber;
  private final String bus;

  public CanDevice(int deviceNumber, String bus) {
    this.deviceNumber = deviceNumber;
    this.bus = bus;
  }

  // Use the default bus name "rio".
  public CanDevice(int deviceNumber) {
    this(deviceNumber, "rio");
  }

  public int getDeviceNumber() {
    return deviceNumber;
  }

  public String getBus() {
    return bus;
  }

  public boolean equals(CanDevice other) {
    return other.deviceNumber == deviceNumber && other.bus.equals(bus);
  }

  @Override
  public String toString() {
    return "CanDeviceId(" + deviceNumber + ", " + bus + ")";
  }
}
