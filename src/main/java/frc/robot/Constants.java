// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;

/** This class contains global configuration describing the current robot and runtime mode. */
public final class Constants {
  public static final double loopPeriodSecs = 0.02;
  private static RobotType robotType = RobotType.SIMBOT;
  public static final boolean tuningMode = false;

  @SuppressWarnings("resource")
  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.kError)
          .set(true);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (robotType) {
      case DEVBOT, COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    SIMBOT,
    DEVBOT,
    COMPBOT
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the correct robot is selected when deploying. */
  /*public static class CheckDeploy {
    public static void main(String... args) {
      if (robotType == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + robotType);
        System.exit(1);
      }
    }
  }*/

  public static final class ReefConstants {
    public enum ReefFaces {
      AB,
      CD,
      EF,
      GH,
      IJ,
      KL
    }

    public enum AlgaeIntakeLocation {
      L2,
      L3
    }

    public static final class AlgaeIntakeMapping {
      public final AlgaeIntakeLocation front;
      public final AlgaeIntakeLocation back;

      public AlgaeIntakeMapping(AlgaeIntakeLocation front, AlgaeIntakeLocation back) {
        this.front = front;
        this.back = back;
      }
    }

    public static final class ScoringCoralMappingRotationToTagID {
      public final int frontId;
      public final int backId;

      public ScoringCoralMappingRotationToTagID(int frontID, int backID) {
        frontId = frontID;
        backId = backID;
      }
    }

    public static final Map<Pose2d, Integer> blueAlliancePoseToTagIdsMap =
        Map.of(
            FieldConstants.getTagPose(21).toPose2d(), 21,
            FieldConstants.getTagPose(20).toPose2d(), 20,
            FieldConstants.getTagPose(19).toPose2d(), 19,
            FieldConstants.getTagPose(18).toPose2d(), 18,
            FieldConstants.getTagPose(17).toPose2d(), 17,
            FieldConstants.getTagPose(22).toPose2d(), 22);

    public static final Map<Pose2d, Integer> redAlliancePoseToTagIdsMap =
        Map.of(
            FieldConstants.getTagPose(6).toPose2d(), 6,
            FieldConstants.getTagPose(7).toPose2d(), 7,
            FieldConstants.getTagPose(8).toPose2d(), 8,
            FieldConstants.getTagPose(9).toPose2d(), 9,
            FieldConstants.getTagPose(10).toPose2d(), 10,
            FieldConstants.getTagPose(11).toPose2d(), 11);

    public static final Map<Rotation2d, ScoringCoralMappingRotationToTagID>
        redAllianceAngleToTagIdsMap =
            Map.of(
                Rotation2d.fromDegrees(-60),
                new ScoringCoralMappingRotationToTagID(9, 6),
                Rotation2d.fromDegrees(-120),
                new ScoringCoralMappingRotationToTagID(8, 11),
                Rotation2d.k180deg,
                new ScoringCoralMappingRotationToTagID(7, 10),
                Rotation2d.fromDegrees(120),
                new ScoringCoralMappingRotationToTagID(6, 9),
                Rotation2d.fromDegrees(60),
                new ScoringCoralMappingRotationToTagID(11, 8),
                Rotation2d.kZero,
                new ScoringCoralMappingRotationToTagID(10, 7));

    public static final Map<Rotation2d, ScoringCoralMappingRotationToTagID>
        blueAllianceAngleToTagIdsMap =
            Map.of(
                Rotation2d.fromDegrees(-60),
                new ScoringCoralMappingRotationToTagID(19, 22),
                Rotation2d.fromDegrees(-120),
                new ScoringCoralMappingRotationToTagID(20, 17),
                Rotation2d.k180deg,
                new ScoringCoralMappingRotationToTagID(21, 18),
                Rotation2d.fromDegrees(120),
                new ScoringCoralMappingRotationToTagID(22, 19),
                Rotation2d.fromDegrees(60),
                new ScoringCoralMappingRotationToTagID(17, 20),
                Rotation2d.kZero,
                new ScoringCoralMappingRotationToTagID(18, 21));

    public static final Map<Rotation2d, AlgaeIntakeMapping> redAllianceAlgae =
        Map.of(
            Rotation2d.fromDegrees(0),
            new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
            Rotation2d.fromDegrees(60),
            new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
            Rotation2d.fromDegrees(120),
            new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
            Rotation2d.fromDegrees(180),
            new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
            Rotation2d.fromDegrees(-120),
            new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
            Rotation2d.fromDegrees(-60),
            new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2));

    public static final Map<Rotation2d, AlgaeIntakeMapping> blueAllianceAlgae =
        Map.of(
            Rotation2d.fromDegrees(0),
            new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
            Rotation2d.fromDegrees(60),
            new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
            Rotation2d.fromDegrees(120),
            new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
            Rotation2d.fromDegrees(180),
            new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
            Rotation2d.fromDegrees(-120),
            new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
            Rotation2d.fromDegrees(-60),
            new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3));

    public static final Map<ReefFaces, Integer> redAllianceReefFacesToIds =
        Map.of(
            ReefFaces.AB, 7,
            ReefFaces.CD, 8,
            ReefFaces.EF, 9,
            ReefFaces.GH, 10,
            ReefFaces.IJ, 11,
            ReefFaces.KL, 6);

    public static final Map<ReefFaces, Integer> blueAllianceReefFacesToIds =
        Map.of(
            ReefFaces.AB, 18,
            ReefFaces.CD, 17,
            ReefFaces.EF, 22,
            ReefFaces.GH, 21,
            ReefFaces.IJ, 20,
            ReefFaces.KL, 19);
  }

  public static final class SysIdConstants {
    public static final Velocity<VoltageUnit> TRANSLATION_RAMP_RATE = null;
    public static final Voltage TRANSLATION_STEP_RATE = Units.Volts.of(7);
    public static final Time TRANSLATION_TIMEOUT = Units.Seconds.of(5);

    /* This is in radians per second², but SysId only supports "volts per second" */
    public static final Velocity<VoltageUnit> ROTATION_RAMP_RATE =
        Units.Volts.of(Math.PI / 6).per(Units.Second);
    /* This is in radians per second, but SysId only supports "volts" */
    public static final Voltage ROTATION_STEP_RATE = Units.Volts.of(Math.PI);
    public static final Time ROTATION_TIMEOUT = Units.Seconds.of(5);

    public static final Velocity<VoltageUnit> STEER_RAMP_RATE = null;
    public static final Voltage STEER_STEP_RATE = Units.Volts.of(7);
    public static final Time STEER_TIMEOUT = null;
  }
}
