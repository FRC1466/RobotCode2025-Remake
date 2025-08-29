# Drive Subsystem

## Overview
The Drive subsystem is built on the CTRE Swerve Drive API and includes several enhancements to support simulation, state-based control, and integration with AdvantageKit. It provides a robust framework for controlling a swerve drivetrain in both teleoperated and autonomous modes.

## Key Features

### 1. **AdvantageKit IO Layers**
- The subsystem uses AdvantageKit's IO layer pattern to separate hardware-specific logic from the subsystem logic.
- Simulation support is built into the CTRE Swerve API, so no additional simulation layers are needed. Simply call `io.updateSimState()` to update the simulation state.

### 2. **State-Based Control**
- **Wanted States**: Define the desired behavior of the drive system (e.g., teleop drive, rotation lock, drive to point, follow choreo path).
- **System States**: Internally computed states that determine how the drive system behaves based on the wanted state.
- State transitions are handled by `handleStateTransitions()`, and the corresponding actions are applied in `applyStates()`.

### 3. **PID Controllers**
- Separate PID controllers for:
  - Choreo path following (`choreoXController`, `choreoYController`, `choreoThetaController`).
  - Drive-to-point alignment in autonomous (`autoDriveToPointController`) and teleop (`teleopDriveToPointController`).

### 4. **Swerve Requests**
- The subsystem uses CTRE's `SwerveRequest` objects to control the drivetrain.
- Supports open-loop voltage control, velocity control, and field-centric heading control.

## Fields and Constants

### General Fields
- `robotWidth`: Used for alignment logic but less relevant in the current implementation.
- `controllerDeadband`: Deadband for joystick inputs, with scaling applied to the remaining range.
- `teleopVelocityCoefficient`: Scales the maximum speed for teleop driving (e.g., for precise maneuvers like barging).
- `rotationVelocityCoefficient`: Scales the maximum angular velocity.

### Error Margins
- `driveToPointTranslationErrorMarginMeters`: Tolerance for determining if the robot is at the desired translation.
- `rotationLockErrorMarginDegrees`: Tolerance for determining if the robot is at the desired rotation.

### State Management
- `wantedState`: The desired state of the drive system.
- `systemState`: The current state being applied.
- `desiredPoseForDriveToPoint`: The target pose for drive-to-point operations.
- `desiredRotationForRotationLockState`: The target rotation for rotation lock.

### IO Inputs
- `swerveInputs`: Contains data for the entire drivetrain.
- `ModuleIOInputs`: Contains data for individual swerve modules (e.g., currents, voltages, temperatures).

## Usage

### Teleop Drive
```java
drive.setWantedState(Drive.WantedState.TELEOP_DRIVE);
drive.setTeleopVelocityCoefficient(1.0); // Full speed
drive.setRotationVelocityCoefficient(1.0); // Full rotation speed
```

### Drive to Point
```java
Pose2d targetPose = new Pose2d(x, y, rotation);
drive.setDesiredPoseForDriveToPoint(targetPose);
drive.setWantedState(Drive.WantedState.DRIVE_TO_POINT);
```

### Rotation Lock
```java
Rotation2d targetRotation = Rotation2d.fromDegrees(90);
drive.setTargetRotation(targetRotation);
drive.setWantedState(Drive.WantedState.ROTATION_LOCK);
```

### Choreo Path Following
```java
Trajectory<SwerveSample> trajectory = ...; // Define the trajectory
drive.setDesiredChoreoTrajectory(trajectory);
drive.setWantedState(Drive.WantedState.CHOREO_PATH);
```

### Simulation
```java
// In simulationPeriodic()
drive.simulationPeriodic();
```

### Vision Measurement Integration
```java
drive.addVisionMeasurement(visionPose, timestampSeconds, visionStdDevsMatrix);
```

### System Identification (SysId)
```java
// Run translation SysId test
drive.sysIdQuasistatic(SysIdMechanism.SWERVE_TRANSLATION, SysIdRoutine.Direction.kForward);
```
