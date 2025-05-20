// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public record SuperstructurePose(DoubleSupplier elevatorHeight, Supplier<Rotation2d> pivotAngle) {
  private static final LoggedTunableNumber StowHeight =
      new LoggedTunableNumber("Superstructure/Stow/Height", 0.00254);
  private static final LoggedTunableNumber StowTravelAngle =
      new LoggedTunableNumber("Superstructure/Stow/TravelSafeAngle", .505);
  private static final LoggedTunableNumber StowRestAngle =
      new LoggedTunableNumber("Superstructure/Stow/RestAngle", 0);

  private static final LoggedTunableNumber intakeHeight =
      new LoggedTunableNumber("Superstructure/Intake/Height", 0.00254);
  private static final LoggedTunableNumber intakeAngle =
      new LoggedTunableNumber("Superstructure/Intake/Angle", 0);

  private static final LoggedTunableNumber IceCreamIntakeHeight =
      new LoggedTunableNumber("Superstructure/AlgaeIntake/IceCream/Height", 0.2);
  private static final LoggedTunableNumber IceCreamIntakeAngle =
      new LoggedTunableNumber("Superstructure/AlgaeIntake/IceCream/Angle", Math.PI);

  private static final LoggedTunableNumber l2ReefIntakeHeight =
      new LoggedTunableNumber("Superstructure/AlgaeIntake/L2/Height", 0.6096);
  private static final LoggedTunableNumber l2ReefIntakeAngle =
      new LoggedTunableNumber("Superstructure/AlgaeIntake/L2/Angle", Math.PI - .3);

  private static final LoggedTunableNumber l3ReefIntakeHeight =
      new LoggedTunableNumber("Superstructure/AlgaeIntake/L3/Height", 1.0668);
  private static final LoggedTunableNumber l3ReefIntakeAngle =
      new LoggedTunableNumber("Superstructure/AlgaeIntake/L3/Angle", Math.PI - .3);

  private static final LoggedTunableNumber NetHeight =
      new LoggedTunableNumber("Superstructure/AlgaeScore/Net/Height", 1.6764);
  private static final LoggedTunableNumber NetAnglePreThrow =
      new LoggedTunableNumber("Superstructure/AlgaeScore/Net/AnglePreThrow", Math.PI - .3);
  private static final LoggedTunableNumber NetAnglePostThrow =
      new LoggedTunableNumber("Superstructure/AlgaeScore/Net/AnglePostThrow", 0);

  private static final LoggedTunableNumber ProcessorHeight =
      new LoggedTunableNumber("Superstructure/AlgaeScore/Processor/Height", 0.11430);
  private static final LoggedTunableNumber ProcessorAngle =
      new LoggedTunableNumber("Superstructure/AlgaeScore/Processor/Angle", Math.PI - .3);

  private static final LoggedTunableNumber l1Height =
      new LoggedTunableNumber("Superstructure/ReefScore/L1/Height", 0.3556);
  private static final LoggedTunableNumber l1Angle =
      new LoggedTunableNumber("Superstructure/ReefScore/L1/Angle", Math.PI - .3);

  private static final LoggedTunableNumber l2Height =
      new LoggedTunableNumber("Superstructure/ReefScore/L2/Height", 0.3556);
  private static final LoggedTunableNumber l2Angle =
      new LoggedTunableNumber("Superstructure/ReefScore/L2/Angle", .505);

  private static final LoggedTunableNumber l3Height =
      new LoggedTunableNumber("Superstructure/ReefScore/L3/Height", 0.7620);
  private static final LoggedTunableNumber l3Angle =
      new LoggedTunableNumber("Superstructure/ReefScore/L3/Angle", .505);

  private static final LoggedTunableNumber l4Height =
      new LoggedTunableNumber("Superstructure/ReefScore/L4/Height", 1.6002);
  private static final LoggedTunableNumber l4Angle =
      new LoggedTunableNumber("Superstructure/ReefScore/L4/Angle", 1.15);

  @Getter
  @RequiredArgsConstructor
  public enum CoralDispenserPose {
    L1(ReefLevel.L1, l1Height, l1Angle),
    L2(ReefLevel.L2, l2Height, l2Angle),
    L3(ReefLevel.L3, l3Height, l3Angle),
    L4(ReefLevel.L4, l4Height, l4Angle);

    private final ReefLevel reefLevel;
    private final LoggedTunableNumber heightSupplier;
    private final LoggedTunableNumber angleSupplier;

    public double getElevatorHeight() {
      return heightSupplier.get();
    }

    public double getManipulatorAngleRad() {
      return angleSupplier.get();
    }
  }

  // Read distance to branch from robot state to calculate positions
  @RequiredArgsConstructor
  @Getter
  enum Preset {
    STOWREST(StowHeight, StowRestAngle),
    STOWTRAVEL(StowHeight, StowTravelAngle),
    CORAL_INTAKE(intakeHeight, intakeAngle),
    L1(CoralDispenserPose.L1),
    L2(CoralDispenserPose.L2),
    L3(CoralDispenserPose.L3),
    L4(CoralDispenserPose.L4),
    ALGAE_L2_INTAKE(l2ReefIntakeHeight, l2ReefIntakeAngle),
    ALGAE_L3_INTAKE(l3ReefIntakeHeight, l3ReefIntakeAngle),
    ALGAE_ICE_CREAM_INTAKE(
        "AlgaeIceCreamIntake", IceCreamIntakeHeight.get(), IceCreamIntakeAngle.get()),
    PRETHROW("Pre-Throw", NetHeight.get(), NetAnglePreThrow.get()),
    THROW("Throw", NetHeight.get(), NetAnglePostThrow.get()),
    // ALGAE_STOW("AlgaeStow", intakeHeightBaseline.get(), -15.0),
    PROCESS("Process", ProcessorHeight.get(), ProcessorAngle.get());

    private final SuperstructurePose pose;

    Preset(DoubleSupplier elevatorHeight, DoubleSupplier pivotAngle) {
      this(
          new SuperstructurePose(
              elevatorHeight, () -> Rotation2d.fromRadians(pivotAngle.getAsDouble())));
    }

    Preset(String name, double elevatorHeight, double pivotAngle) {
      this(
          new LoggedTunableNumber("Superstructure/" + name + "/Elevator", elevatorHeight),
          new LoggedTunableNumber("Superstructure/" + name + "/Pivot", pivotAngle));
    }

    Preset(CoralDispenserPose coralDispenserPose) {
      pose =
          new SuperstructurePose(
              coralDispenserPose::getElevatorHeight,
              () -> Rotation2d.fromRadians(coralDispenserPose.getManipulatorAngleRad()));
    }
  }
}
