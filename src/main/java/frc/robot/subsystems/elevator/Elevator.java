// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.constants.ElevatorConstants.*;

import java.util.function.Supplier;

import lombok.Getter;
import lombok.Setter;

public class Elevator {
     @Getter @Setter private static Supplier<ElevatorProfile> currentElevatorProfile =
      () -> ElevatorProfile.DEFAULT;

  public enum ElevatorProfile {
    DEFAULT(velocityConstraint, accelerationConstraint),
    DOWN(velocityConstraint, accelerationConstraintDown),
    ALGAE(velocityConstraintAlgae, accelerationConstraintAlgae);

    public final double velocity;
    public final double acceleration;

    ElevatorProfile(double velocity, double acceleration) {
      this.velocity = velocity;
      this.acceleration = acceleration;
    }
  }
}
