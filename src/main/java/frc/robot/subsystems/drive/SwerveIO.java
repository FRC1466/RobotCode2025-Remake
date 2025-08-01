// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {
  @AutoLog
  class SwerveIOInputs {
    public Pose2d Pose = new Pose2d();
    public ChassisSpeeds Speeds = new ChassisSpeeds();
    public SwerveModuleState[] ModuleStates;
    public SwerveModuleState[] ModuleTargets;
    public SwerveModulePosition[] ModulePositions;
    public Rotation2d RawHeading = new Rotation2d();
    public double Timestamp;
    public double OdometryPeriod;
    public int SuccessfulDaqs;
    public int FailedDaqs;

    void logState(SwerveDrivetrain.SwerveDriveState state) {
      this.Pose = state.Pose;
      this.RawHeading = state.RawHeading;
      this.ModuleStates = state.ModuleStates;
      this.ModuleTargets = state.ModuleTargets;
      this.ModulePositions = state.ModulePositions;
      this.Speeds = state.Speeds;
      this.SuccessfulDaqs = state.SuccessfulDaqs;
      this.FailedDaqs = state.FailedDaqs;
      this.OdometryPeriod = state.OdometryPeriod;
      this.Timestamp = state.OdometryPeriod;
      RobotState.getInstance()
          .addPoseObservation(new RobotState.SwerveDriveObservation(this.Pose, this.Speeds));
    }
  }

  @AutoLog
  class ModuleIOInputs {
    public double driveSupplyCurrentAmps = 0.0;
    public double driveStatorCurrentAmps = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveTemperature = 0.0;

    public double steerSupplyCurrentAmps = 0.0;
    public double steerStatorCurrentAmps = 0.0;
    public double steerAppliedVolts = 0.0;
    public double steerTemperature = 0.0;
  }

  default void updateSwerveInputs(SwerveIOInputs inputs) {}

  default void updateModuleInputs(ModuleIOInputs... inputs) {}

  default void registerTelemetryFunction(SwerveIOInputs inputs) {}

  default void setSwerveState(SwerveRequest request) {}

  default void resetRotation() {}

  default void resetToParamaterizedRotation(Rotation2d rotation2d) {}

  default void updateSimState() {}

  default void resetRobotTranslation(Translation2d translation2d) {}

  default void addVisionMeasurement(
      Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> visionStdDevs) {}
}
