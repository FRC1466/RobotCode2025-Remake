// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;

public class SystemTimeValidReader {
  private static Thread thread = null;
  private static boolean ready = false;

  public static void start() {
    if (thread != null) return;
    thread =
        new Thread(
            () -> {
              while (true) {
                boolean readyNew = RobotController.isSystemTimeValid();
                synchronized (SystemTimeValidReader.class) {
                  ready = readyNew;
                }
                try {
                  Thread.sleep(3000);
                } catch (InterruptedException e) {
                  e.printStackTrace();
                }
              }
            });
    thread.setName("SystemTimeValidReader");
    thread.start();
  }

  public static synchronized boolean isValid() {
    return ready;
  }
}
