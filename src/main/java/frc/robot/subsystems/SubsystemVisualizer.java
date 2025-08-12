// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SubsystemVisualizer extends SubsystemBase {

  // Zero-position rotational offsets (radians)
  private static final double yaw1OffsetRad = 0.0;
  private static final double yaw4OffsetRad = 0.0;

  private final DoubleSupplier elevatorHeightMeters;
  private final DoubleSupplier armAngleRadians; // used on component 4 (Z axis)
  private final DoubleSupplier intakeAngleRadians; // used on component 1 (Y axis)
  private final BooleanSupplier hasCoral;
  private final BooleanSupplier hasAlgae;
  private final BooleanSupplier coralInIntake;
  private final Supplier<Pose2d> robotPose;

  // Tunables
  private final LoggedTunableNumber stage1Max;
  private final LoggedTunableNumber stage2Max;

  private final LoggedTunableNumber algaeXOffset;
  private final LoggedTunableNumber algaeYOffset;
  private final LoggedTunableNumber algaeZOffset;

  private final LoggedTunableNumber coralXOffset;
  private final LoggedTunableNumber coralYOffset;
  private final LoggedTunableNumber coralZOffset;
  private final LoggedTunableNumber coralPitchDeg;
  private final LoggedTunableNumber coralYawDeg;
  private final LoggedTunableNumber coralRollDeg;

  private final LoggedTunableNumber coralXOffsetWrist;
  private final LoggedTunableNumber coralYOffsetWrist;
  private final LoggedTunableNumber coralZOffsetWrist;
  private final LoggedTunableNumber coralPitchDegWrist;
  private final LoggedTunableNumber coralYawDegWrist;
  private final LoggedTunableNumber coralRollDegWrist;

  private final String name;

  public SubsystemVisualizer(
      String name,
      DoubleSupplier elevatorHeightMeters,
      DoubleSupplier armAngleRadians,
      DoubleSupplier intakeAngleRadians,
      BooleanSupplier hasCoral,
      BooleanSupplier hasAlgae,
      BooleanSupplier coralInIntake,
      Supplier<Pose2d> robotPose) {
    this.name = name;
    this.elevatorHeightMeters = elevatorHeightMeters;
    this.armAngleRadians = armAngleRadians;
    this.intakeAngleRadians = intakeAngleRadians;
    this.hasCoral = hasCoral;
    this.hasAlgae = hasAlgae;
    this.coralInIntake = coralInIntake;
    this.robotPose = robotPose;

    String prefix = "Visualizer/" + name + "/";

    stage1Max = new LoggedTunableNumber(prefix + "Stage1MaxMeters", .85);
    stage2Max = new LoggedTunableNumber(prefix + "Stage2MaxMeters", .657);

    algaeXOffset = new LoggedTunableNumber(prefix + "AlgaeXOffset", .23);
    algaeYOffset = new LoggedTunableNumber(prefix + "AlgaeYOffset", 0.0);
    algaeZOffset = new LoggedTunableNumber(prefix + "AlgaeZOffset", 0.69);

    coralXOffset = new LoggedTunableNumber(prefix + "CoralXOffset", -0.093431);
    coralYOffset = new LoggedTunableNumber(prefix + "CoralYOffset", 0.0);
    coralZOffset = new LoggedTunableNumber(prefix + "CoralZOffset", 0.211419);
    coralPitchDeg = new LoggedTunableNumber(prefix + "CoralPitchDeg", 0);
    coralYawDeg = new LoggedTunableNumber(prefix + "CoralYawDeg", 90);
    coralRollDeg = new LoggedTunableNumber(prefix + "CoralRollDeg", 0);

    coralXOffsetWrist = new LoggedTunableNumber(prefix + "CoralXOffsetWrist", .24);
    coralYOffsetWrist = new LoggedTunableNumber(prefix + "CoralYOffsetWrist", 0.0);
    coralZOffsetWrist = new LoggedTunableNumber(prefix + "CoralZOffsetWrist", 0.542219);
    coralPitchDegWrist = new LoggedTunableNumber(prefix + "CoralPitchDegWrist", 0);
    coralYawDegWrist = new LoggedTunableNumber(prefix + "CoralYawDegWrist", 90);
    coralRollDegWrist = new LoggedTunableNumber(prefix + "CoralRollDegWrist", 0);
  }

  @Override
  public void periodic() {
    update3dVisuals(
        elevatorHeightMeters.getAsDouble(),
        armAngleRadians.getAsDouble(),
        intakeAngleRadians.getAsDouble(),
        hasAlgae.getAsBoolean(),
        hasCoral.getAsBoolean(),
        coralInIntake.getAsBoolean(),
        robotPose.get());
  }

  private void update3dVisuals(
      double elevatorHeightMeters,
      double armAngleRadians,
      double intakeAngleRadians,
      boolean hasAlgae,
      boolean hasCoral,
      boolean coralInIntake,
      Pose2d robotPose) {

    double stage1MaxVal = stage1Max.get();
    double stage2MaxVal = stage2Max.get();

    double totalVisualMaxHeight = stage1MaxVal + stage2MaxVal;
    double h = Math.max(0.0, Math.min(elevatorHeightMeters, totalVisualMaxHeight));

    double stage2Travel = Math.min(h, stage2MaxVal);
    double stage1Travel = Math.max(0.0, Math.min(h - stage2MaxVal, stage1MaxVal));

    Pose3d p1 =
        new Pose3d(
            0.330200, 0.0, 0.171450, new Rotation3d(0.0, yaw1OffsetRad + intakeAngleRadians, 0.0));

    Pose3d p2 = new Pose3d(0.0, 0.0, stage1Travel, new Rotation3d());

    Pose3d p3 = new Pose3d(0.0, 0.0, stage2Travel, new Rotation3d());

    Pose3d p4 =
        new Pose3d(0.0, 0.0, 0.225, new Rotation3d(yaw4OffsetRad + armAngleRadians, 0.0, 0.0));

    Pose3d component1Pose = p1;
    Pose3d component2Pose = p2;
    Pose3d component3Pose = component2Pose.transformBy(new Transform3d(Pose3d.kZero, p3));
    Pose3d component4Pose = component3Pose.transformBy(new Transform3d(Pose3d.kZero, p4));

    Logger.recordOutput(
        "Mechanism3d/" + name + "/Components",
        component1Pose,
        component2Pose,
        component3Pose,
        component4Pose);

    if (hasAlgae) {
      Logger.recordOutput(
          "Mechanism3d/" + name + "/Algae",
          new Translation3d[] {
            new Pose3d(robotPose)
                .transformBy(new Transform3d(Pose3d.kZero, component4Pose))
                .transformBy(
                    new Transform3d(
                        algaeXOffset.get(),
                        algaeYOffset.get(),
                        algaeZOffset.get(),
                        new Rotation3d()))
                .getTranslation()
          });
    } else {
      Logger.recordOutput("Mechanism3d/" + name + "/Algae", new Translation3d[] {});
    }

    if (hasCoral) {
      if (coralInIntake) {
        Logger.recordOutput(
            "Mechanism3d/" + name + "/Coral",
            new Pose3d[] {
              new Pose3d(robotPose)
                  .transformBy(new Transform3d(Pose3d.kZero, component1Pose))
                  .transformBy(
                      new Transform3d(
                          coralXOffset.get(),
                          coralYOffset.get(),
                          coralZOffset.get(),
                          new Rotation3d(
                              Units.degreesToRadians(coralRollDeg.get()),
                              Units.degreesToRadians(coralPitchDeg.get()),
                              Units.degreesToRadians(coralYawDeg.get()))))
            });
      } else {
        Logger.recordOutput(
            "Mechanism3d/" + name + "/Coral",
            new Pose3d[] {
              new Pose3d(robotPose)
                  .transformBy(new Transform3d(Pose3d.kZero, component4Pose))
                  .transformBy(
                      new Transform3d(
                          coralXOffsetWrist.get(),
                          coralYOffsetWrist.get(),
                          coralZOffsetWrist.get(),
                          new Rotation3d(
                              Units.degreesToRadians(coralRollDegWrist.get()),
                              Units.degreesToRadians(coralPitchDegWrist.get()),
                              Units.degreesToRadians(coralYawDegWrist.get()))))
            });
      }
    } else {
      Logger.recordOutput("Mechanism3d/" + name + "/Coral", new Pose3d[] {});
    }
  }
}
