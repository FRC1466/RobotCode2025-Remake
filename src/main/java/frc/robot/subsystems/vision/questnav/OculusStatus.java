// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision.questnav;

import lombok.experimental.UtilityClass;

@UtilityClass
public final class OculusStatus {

  public class Miso {
    /** Status indicating system is ready for commands */
    public static final int STATUS_READY = 0;

    /** Status indicating heading reset completion */
    public static final int STATUS_HEADING_RESET_COMPLETE = 99;

    /** Status indicating pose reset completion */
    public static final int STATUS_POSE_RESET_COMPLETE = 98;

    /** Status indicating ping response receipt */
    public static final int STATUS_PING_RESPONSE = 97;
  }

  public class Mosi {
    /** Clear status */
    public static final int COMMAND_CLEAR = 0;

    /** Command to reset the heading */
    public static final int COMMAND_RESET_HEADING = 1;

    /** Command to reset the pose */
    public static final int COMMAND_RESET_POSE = 2;

    /** Command to ping the system */
    public static final int COMMAND_PING = 3;
  }
}
