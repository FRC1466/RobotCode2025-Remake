// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class WristElevatorPoses {
  public static final WristElevatorPose STOW = new WristElevatorPose(Rotation2d.fromDegrees(0), 0);

  public static final WristElevatorPose TRAVEL =
      new WristElevatorPose(Rotation2d.fromDegrees(40), 0.05);

  public static final WristElevatorPose L1 =
      new WristElevatorPose(Rotation2d.fromRadians(Math.PI - 0.67), 0.5);

  public static final WristElevatorPose L2 =
      new WristElevatorPose(Rotation2d.fromDegrees(40), 0.38);

  public static final WristElevatorPose L3 = new WristElevatorPose(Rotation2d.fromDegrees(40), 0.8);

  public static final WristElevatorPose L4 =
      new WristElevatorPose(Rotation2d.fromDegrees(67), 1.15);

  public static final WristElevatorPose L2_ALGAE =
      new WristElevatorPose(Rotation2d.fromRadians(Math.PI - .3), 0.6);

  public static final WristElevatorPose L3_ALGAE =
      new WristElevatorPose(Rotation2d.fromRadians(Math.PI - .3), 1.0);

  public static final WristElevatorPose ALGAE_PROCESSOR =
      new WristElevatorPose(Rotation2d.fromRadians(Math.PI - .3), 0.1);

  public static final WristElevatorPose ALGAE_NET_PRE =
      new WristElevatorPose(Rotation2d.fromRadians(Math.PI - .3), 1.67);

  public static final WristElevatorPose ALGAE_NET_POST =
      new WristElevatorPose(Rotation2d.fromDegrees(0), 1.67);

  public static final WristElevatorPose ALGAE_HOLD =
      new WristElevatorPose(Rotation2d.fromRadians(Math.PI - .3), .15);

  public static final WristElevatorPose ALGAE_HOLD_CLOSER =
      new WristElevatorPose(Rotation2d.fromDegrees(140), 0.05);

  public static class WristElevatorPose {
    public final Rotation2d wristAngle;
    public final double elevatorHeight;

    public WristElevatorPose(Rotation2d wristAngle, double elevatorHeightMeters) {
      this.wristAngle = wristAngle;
      this.elevatorHeight = elevatorHeightMeters;
    }
  }
}
