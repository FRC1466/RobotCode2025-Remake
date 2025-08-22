// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.SysIDConstants.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.util.AllianceUtil;
import frc.robot.util.LoggedTracer;
import frc.robot.util.SysIdMechanism;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  /** Includes bumpers! */
  public static final double robotWidth =
      Units.inchesToMeters(28.0) + 2 * Units.inchesToMeters(2.0);

  private static final double controllerDeadband = 0.1;

  public static final double releasePieceTranslationErrorMarginMeters = Units.inchesToMeters(0.5);
  public static final double driveToPointTranslationErrorMarginMeters = Units.inchesToMeters(1.0);

  public static final double driveToPointStaticFrictionCompensation = 0.02;

  public static final double rotationLockErrorMarginDegrees = 10.0;

  public final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
  public final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
  public final SlewRateLimiter thetaLimiter = new SlewRateLimiter(3.0);

  public double maxVelocityOutputForDriveToPoint = Units.feetToMeters(10.0);

  private final PIDController choreoXController = new PIDController(7, 0, 0);
  private final PIDController choreoYController = new PIDController(7, 0, 0);
  private final PIDController choreoThetaController = new PIDController(7, 0, 0);

  private final PIDController autoDriveToPointController = new PIDController(3.0, 0, 0.1);
  private final PIDController teleopDriveToPointController = new PIDController(3.6, 0, 0.1);

  private final SwerveRequest.SysIdSwerveTranslation translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveRotation rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();
  private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();

  private final SwerveRequest.FieldCentricFacingAngle driveAtAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  private static final double skewCompensationScalar = -0.03;

  public enum WantedState {
    SYS_ID,
    TELEOP_DRIVE,
    CHOREO_PATH,
    ROTATION_LOCK,
    DRIVE_TO_POINT,
    IDLE
  }

  public enum SystemState {
    SYS_ID,
    TELEOP_DRIVE,
    CHOREO_PATH,
    ROTATION_LOCK,
    DRIVE_TO_POINT,
    IDLE
  }

  @Getter private SystemState systemState = SystemState.TELEOP_DRIVE;
  @Getter private WantedState wantedState = WantedState.TELEOP_DRIVE;

  private Rotation2d desiredRotationForRotationLockState;

  private Trajectory<SwerveSample> desiredChoreoTrajectory;
  private final Timer choreoTimer = new Timer();
  private Optional<SwerveSample> choreoSampleToBeApplied;

  private Pose2d desiredPoseForDriveToPoint = new Pose2d();

  final DriveIOInputsAutoLogged swerveInputs = new DriveIOInputsAutoLogged();
  ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged();
  ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged();
  ModuleIOInputsAutoLogged backLeftInputs = new ModuleIOInputsAutoLogged();
  ModuleIOInputsAutoLogged backRightInputs = new ModuleIOInputsAutoLogged();

  private final Object moduleIOLock = new Object();

  private DriveIO io = new DriveIO() {};
  private final CommandXboxController controller;

  private final double maxVelocity;
  private final double maxAngularVelocity;

  private double teleopVelocityCoefficient = 1.0;
  private double rotationVelocityCoefficient = 1.0;
  private double maximumAngularVelocityForDriveToPoint = Double.NaN;

  private final SysIdRoutine translationSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              translationRampRate,
              translationStepRate,
              translationTimeout,
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> io.setSwerveState(translationCharacterization.withVolts(output)),
              null,
              this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   */
  private final SysIdRoutine rotationSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              rotationRampRate,
              rotationStepRate,
              rotationTimeout,
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                io.setSwerveState(rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  private final SysIdRoutine steerSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              steerRampRate,
              steerStepRate,
              steerTimeout,
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> io.setSwerveState(steerCharacterization.withVolts(volts)), null, this));

  public Drive(
      DriveIO io, CommandXboxController controller, double maxVelocity, double maxAngularVelocity) {
    this.io = io;
    this.controller = controller;
    this.maxVelocity = maxVelocity;
    this.maxAngularVelocity = maxAngularVelocity;

    // io.registerTelemetryFunction(swerveInputs);
    driveAtAngle.HeadingController = new PhoenixPIDController(5, 0, 0);

    driveAtAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Runs the quasi-static SysId test in the given direction for the given mechanism.
   *
   * @param mechanism The mechanism to characterize
   * @param direction Direction of the quasi-static SysId test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdMechanism mechanism, SysIdRoutine.Direction direction) {
    final SysIdRoutine routine =
        switch (mechanism) {
          case SWERVE_TRANSLATION -> translationSysIdRoutine;
          case SWERVE_ROTATION -> rotationSysIdRoutine;
          case SWERVE_STEER -> steerSysIdRoutine;
          default ->
              throw new IllegalArgumentException(
                  String.format("Mechanism %s is not supported.", mechanism));
        };

    return routine.quasistatic(direction);
  }

  /**
   * Runs the dynamic SysId test in the given direction for the given mechanism.
   *
   * @param mechanism The mechanism to characterize
   * @param direction Direction of the dynamic SysId test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdMechanism mechanism, SysIdRoutine.Direction direction) {
    final SysIdRoutine routine =
        switch (mechanism) {
          case SWERVE_TRANSLATION -> translationSysIdRoutine;
          case SWERVE_ROTATION -> rotationSysIdRoutine;
          case SWERVE_STEER -> steerSysIdRoutine;
          default ->
              throw new IllegalArgumentException(
                  String.format("Mechanism %s is not supported.", mechanism));
        };

    return routine.dynamic(direction);
  }

  @Override
  public void periodic() {
    io.updateSwerveInputs(swerveInputs);
    synchronized (moduleIOLock) {
      io.updateModuleInputs(frontLeftInputs, frontRightInputs, backLeftInputs, backRightInputs);
      Logger.processInputs("Subsystems/Drive", swerveInputs);
      Logger.processInputs("Subsystems/Drive/Module Data/Front Left", frontLeftInputs);
      Logger.processInputs("Subsystems/Drive/Module Data/Front Right", frontRightInputs);
      Logger.processInputs("Subsystems/Drive/Module Data/Back Left", backLeftInputs);
      Logger.processInputs("Subsystems/Drive/Module Data/Back Right", backRightInputs);
    }

    systemState = handleStateTransition();

    Logger.recordOutput("Subsystems/Drive/SystemState", systemState);
    Logger.recordOutput("Subsystems/Drive/DesiredState", wantedState);
    applyStates();

    // Record cycle time
    LoggedTracer.record("Drive");
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case SYS_ID -> SystemState.SYS_ID;
      case TELEOP_DRIVE -> SystemState.TELEOP_DRIVE;
      case CHOREO_PATH -> {
        if (systemState != SystemState.CHOREO_PATH) {
          choreoTimer.restart();
          choreoSampleToBeApplied = desiredChoreoTrajectory.sampleAt(choreoTimer.get(), false);
          yield SystemState.CHOREO_PATH;
        } else {
          choreoSampleToBeApplied = desiredChoreoTrajectory.sampleAt(choreoTimer.get(), false);
          yield SystemState.CHOREO_PATH;
        }
      }
      case ROTATION_LOCK -> SystemState.ROTATION_LOCK;
      case DRIVE_TO_POINT -> SystemState.DRIVE_TO_POINT;
      default -> SystemState.IDLE;
    };
  }

  private void applyStates() {
    switch (systemState) {
      default:
      case SYS_ID:
        break;
      case TELEOP_DRIVE:
        io.setSwerveState(
            new SwerveRequest.ApplyFieldSpeeds()
                .withSpeeds(calculateSpeedsBasedOnJoystickInputs())
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
        break;
      case CHOREO_PATH:
        {
          if (choreoSampleToBeApplied.isPresent()) {
            var sample = choreoSampleToBeApplied.get();
            Logger.recordOutput("Subsystems/Drive/Choreo/Timer Value", choreoTimer.get());
            Logger.recordOutput(
                "Subsystems/Drive/Choreo/Traj Name", desiredChoreoTrajectory.name());
            Logger.recordOutput(
                "Subsystems/Drive/Choreo/Total time", desiredChoreoTrajectory.getTotalTime());
            Logger.recordOutput("Subsystems/Drive/Choreo/sample/Desired Pose", sample.getPose());
            Logger.recordOutput(
                "Subsystems/Drive/Choreo/sample/Desired Chassis Speeds", sample.getChassisSpeeds());
            Logger.recordOutput(
                "Subsystems/Drive/Choreo/sample/Module Forces X", sample.moduleForcesX());
            Logger.recordOutput(
                "Subsystems/Drive/Choreo/sample/Module Forces Y", sample.moduleForcesY());
            synchronized (swerveInputs) {
              var pose = swerveInputs.Pose;

              var targetSpeeds = sample.getChassisSpeeds();
              targetSpeeds.vxMetersPerSecond += choreoXController.calculate(pose.getX(), sample.x);
              targetSpeeds.vyMetersPerSecond += choreoYController.calculate(pose.getY(), sample.y);
              targetSpeeds.omegaRadiansPerSecond +=
                  choreoThetaController.calculate(pose.getRotation().getRadians(), sample.heading);

              io.setSwerveState(
                  new SwerveRequest.ApplyFieldSpeeds()
                      .withSpeeds(targetSpeeds)
                      .withWheelForceFeedforwardsX(sample.moduleForcesX())
                      .withWheelForceFeedforwardsY(sample.moduleForcesY())
                      .withDriveRequestType(SwerveModule.DriveRequestType.Velocity));
            }
          }
          break;
        }
      case ROTATION_LOCK:
        io.setSwerveState(
            driveAtAngle
                .withVelocityX(calculateSpeedsBasedOnJoystickInputs().vxMetersPerSecond)
                .withVelocityY(calculateSpeedsBasedOnJoystickInputs().vyMetersPerSecond)
                .withTargetDirection(desiredRotationForRotationLockState));
        break;
      case DRIVE_TO_POINT:
        var translationToDesiredPoint =
            desiredPoseForDriveToPoint.getTranslation().minus(swerveInputs.Pose.getTranslation());
        var linearDistance = translationToDesiredPoint.getNorm();
        var frictionConstant = 0.0;
        if (linearDistance >= Units.inchesToMeters(0.5)) {
          frictionConstant = driveToPointStaticFrictionCompensation * maxVelocity;
        }
        var directionOfTravel = translationToDesiredPoint.getAngle();
        var velocityOutput = 0.0;
        if (DriverStation.isAutonomous()) {
          velocityOutput =
              Math.min(
                  Math.abs(autoDriveToPointController.calculate(linearDistance, 0))
                      + frictionConstant,
                  maxVelocityOutputForDriveToPoint);
        } else {
          velocityOutput =
              Math.min(
                  Math.abs(teleopDriveToPointController.calculate(linearDistance, 0))
                      + frictionConstant,
                  maxVelocityOutputForDriveToPoint);
        }
        var xComponent = velocityOutput * directionOfTravel.getCos();
        var yComponent = velocityOutput * directionOfTravel.getSin();

        Logger.recordOutput("Subsystems/Drive/DriveToPoint/xVelocitySetpoint", xComponent);
        Logger.recordOutput("Subsystems/Drive/DriveToPoint/yVelocitySetpoint", yComponent);
        Logger.recordOutput("Subsystems/Drive/DriveToPoint/velocityOutput", velocityOutput);
        Logger.recordOutput("Subsystems/Drive/DriveToPoint/linearDistance", linearDistance);
        Logger.recordOutput("Subsystems/Drive/DriveToPoint/directionOfTravel", directionOfTravel);
        Logger.recordOutput(
            "Subsystems/Drive/DriveToPoint/desiredPoint", desiredPoseForDriveToPoint);

        if (Double.isNaN(maximumAngularVelocityForDriveToPoint)) {
          io.setSwerveState(
              driveAtAngle
                  .withVelocityX(xComponent)
                  .withVelocityY(yComponent)
                  .withTargetDirection(desiredPoseForDriveToPoint.getRotation()));
        } else {
          io.setSwerveState(
              driveAtAngle
                  .withVelocityX(xComponent)
                  .withVelocityY(yComponent)
                  .withTargetDirection(desiredPoseForDriveToPoint.getRotation())
                  .withMaxAbsRotationalRate(maximumAngularVelocityForDriveToPoint));
        }
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    synchronized (moduleIOLock) {
      io.updateSimState();
    }
  }

  public void setState(WantedState state) {
    this.wantedState = state;
  }

  public void setDesiredChoreoTrajectory(Trajectory<SwerveSample> trajectory) {
    this.desiredChoreoTrajectory = trajectory;
    this.wantedState = WantedState.CHOREO_PATH;
    choreoTimer.reset();
  }

  public void setDesiredPoseForDriveToPoint(Pose2d pose) {
    this.desiredPoseForDriveToPoint = pose;
    this.wantedState = WantedState.DRIVE_TO_POINT;
    this.maxVelocityOutputForDriveToPoint = Units.feetToMeters(10.0);
    this.maximumAngularVelocityForDriveToPoint = Double.NaN;
  }

  public void setDesiredPoseForDriveToPointWithMaximumAngularVelocity(
      Pose2d pose, double maximumAngularVelocityForDriveToPoint) {
    this.desiredPoseForDriveToPoint = pose;
    this.wantedState = WantedState.DRIVE_TO_POINT;
    this.maxVelocityOutputForDriveToPoint = Units.feetToMeters(10.0);
    this.maximumAngularVelocityForDriveToPoint = maximumAngularVelocityForDriveToPoint;
  }

  public void setDesiredPoseForDriveToPoint(Pose2d pose, double maxVelocityOutputForDriveToPoint) {
    this.desiredPoseForDriveToPoint = pose;
    this.wantedState = WantedState.DRIVE_TO_POINT;
    this.maxVelocityOutputForDriveToPoint = maxVelocityOutputForDriveToPoint;
  }

  public void setDesiredPoseForDriveToPointWithConstraints(
      Pose2d pose,
      double maxVelocityOutputForDriveToPoint,
      double maximumAngularVelocityForDriveToPoint) {
    this.desiredPoseForDriveToPoint = pose;
    this.wantedState = WantedState.DRIVE_TO_POINT;
    this.maxVelocityOutputForDriveToPoint = maxVelocityOutputForDriveToPoint;
    this.maximumAngularVelocityForDriveToPoint = maximumAngularVelocityForDriveToPoint;
  }

  private ChassisSpeeds calculateSpeedsBasedOnJoystickInputs() {
    if (DriverStation.getAlliance().isEmpty()) {
      return new ChassisSpeeds(0, 0, 0);
    }

    double xMagnitude = MathUtil.applyDeadband(controller.getLeftY(), controllerDeadband);
    double yMagnitude = MathUtil.applyDeadband(controller.getLeftX(), controllerDeadband);
    double angularMagnitude = MathUtil.applyDeadband(controller.getRightX(), controllerDeadband);

    // xMagnitude = xLimiter.calculate(xMagnitude);
    // yMagnitude = yLimiter.calculate(yMagnitude);

    // Commented out, enable for smoother driving
    // xMagnitude = Math.copySign(xMagnitude * xMagnitude, xMagnitude);
    // yMagnitude = Math.copySign(yMagnitude * yMagnitude, yMagnitude);
    angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

    double xVelocity =
        (AllianceUtil.isBlueAlliance() ? -xMagnitude * maxVelocity : xMagnitude * maxVelocity)
            * teleopVelocityCoefficient;
    double yVelocity =
        (AllianceUtil.isBlueAlliance() ? -yMagnitude * maxVelocity : yMagnitude * maxVelocity)
            * teleopVelocityCoefficient;
    double angularVelocity = angularMagnitude * maxAngularVelocity * rotationVelocityCoefficient;

    Rotation2d skewCompensationFactor =
        Rotation2d.fromRadians(swerveInputs.Speeds.omegaRadiansPerSecond * skewCompensationScalar);

    return ChassisSpeeds.fromRobotRelativeSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(xVelocity, yVelocity, -angularVelocity),
            swerveInputs.Pose.getRotation()),
        swerveInputs.Pose.getRotation().plus(skewCompensationFactor));
  }

  public void resetTranslationAndRotation(Pose2d pose2d) {
    resetTranslation(pose2d);
    resetRotation(pose2d.getRotation());
  }

  public void resetRotation(Rotation2d rotation2d) {
    io.resetToParamaterizedRotation(rotation2d);
  }

  public void resetTranslation(Pose2d pose) {
    io.resetRobotTranslation(pose.getTranslation());
  }

  public void resetRotationBasedOnAlliance() {
    io.resetRotation();
  }

  public void setWantedState(WantedState state) {
    this.wantedState = state;
  }

  public void setTargetRotation(Rotation2d desiredRotation) {
    setWantedState(WantedState.ROTATION_LOCK);
    this.desiredRotationForRotationLockState = desiredRotation;
  }

  public boolean isStopped() {
    return isStopped(0.01);
  }

  public boolean isStopped(double tolerance) {
    return Math.abs(swerveInputs.Speeds.vxMetersPerSecond) < tolerance
        && Math.abs(swerveInputs.Speeds.vyMetersPerSecond) < tolerance
        && Math.abs(swerveInputs.Speeds.omegaRadiansPerSecond) < tolerance;
  }

  public boolean isAtDriveToPointSetpoint(
      double translationTolerance, Rotation2d rotationTolerance) {
    var translationDistance =
        desiredPoseForDriveToPoint
            .getTranslation()
            .minus(swerveInputs.Pose.getTranslation())
            .getNorm();
    var rotationError =
        Math.abs(
            desiredPoseForDriveToPoint
                .getRotation()
                .minus(swerveInputs.Pose.getRotation())
                .getRadians());

    Logger.recordOutput("Subsystems/Drive/DriveToPoint/distanceFromEndpoint", translationDistance);
    Logger.recordOutput("Subsystems/Drive/DriveToPoint/rotationError", rotationError);

    return MathUtil.isNear(0.0, translationDistance, translationTolerance)
        && rotationError < rotationTolerance.getRadians();
  }

  public boolean isAtDriveToPointSetpoint() {
    return isAtDriveToPointSetpoint(
        driveToPointTranslationErrorMarginMeters,
        Rotation2d.fromDegrees(rotationLockErrorMarginDegrees));
  }

  public boolean isAtDesiredRotation() {
    return isAtDesiredRotation(Units.degreesToRadians(10.0));
  }

  public boolean isAtDesiredRotation(double tolerance) {
    return driveAtAngle.HeadingController.getPositionError() < tolerance;
  }

  public boolean isAtChoreoSetpoint() {
    if (systemState != SystemState.CHOREO_PATH) {
      return false;
    }
    return MathUtil.isNear(
            desiredChoreoTrajectory.getFinalPose(false).get().getX(),
            swerveInputs.Pose.getX(),
            releasePieceTranslationErrorMarginMeters)
        && MathUtil.isNear(
            desiredChoreoTrajectory.getFinalPose(false).get().getY(),
            swerveInputs.Pose.getY(),
            releasePieceTranslationErrorMarginMeters);
  }

  public boolean isAtEndOfChoreoTrajectoryOrDriveToPoint() {
    if (desiredChoreoTrajectory != null) {
      return (MathUtil.isNear(
                  desiredChoreoTrajectory.getFinalPose(false).get().getX(),
                  swerveInputs.Pose.getX(),
                  releasePieceTranslationErrorMarginMeters)
              && MathUtil.isNear(
                  desiredChoreoTrajectory.getFinalPose(false).get().getY(),
                  swerveInputs.Pose.getY(),
                  releasePieceTranslationErrorMarginMeters))
          || isAtDriveToPointSetpoint();
    } else {
      return isAtDriveToPointSetpoint();
    }
  }

  public double getRobotDistanceFromChoreoEndpoint() {
    Logger.recordOutput("Choreo/FinalPose", desiredChoreoTrajectory.getFinalPose(false).get());
    var distance =
        Math.abs(
            desiredChoreoTrajectory
                .getFinalPose(false)
                .get()
                .minus(RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry())
                .getTranslation()
                .getNorm());
    Logger.recordOutput("Choreo/DistanceFromEndpoint", distance);
    return distance;
  }

  public double getDistanceFromDriveToPointSetpoint() {
    var diff =
        desiredPoseForDriveToPoint
            .getTranslation()
            .minus(swerveInputs.Pose.getTranslation())
            .getNorm();
    Logger.recordOutput("DistanceFromDriveToPointSetpoint", diff);
    return diff;
  }

  public void setTeleopVelocityCoefficient(double teleopVelocityCoefficient) {
    this.teleopVelocityCoefficient = teleopVelocityCoefficient;
  }

  public void setRotationVelocityCoefficient(double rotationVelocityCoefficient) {
    this.rotationVelocityCoefficient = rotationVelocityCoefficient;
  }

  public void addVisionMeasurement(
      Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> visionStdDevs) {
    io.addVisionMeasurement(visionPose, timestampSeconds, visionStdDevs);
  }
}
