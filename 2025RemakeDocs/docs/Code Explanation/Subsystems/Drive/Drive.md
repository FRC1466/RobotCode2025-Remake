# Drive.Java
## Definition
The Drive Subsystem is based off the CTRE swerve drive API. However, it has been adapted to contain a few major addtions:
1. AdvantageKit IO Layers: Due to the CTRE Swerve API Simulation support, no additional IO Layer is needed for simulation, you just call io.updateSimState();
2. System and Wanted States: By setting the Drive wanted state to drive, drive at rotation, drive to point, or follow choreo path
## Fields
- robotWidth: was used for auto align logic to go the correct distance, but isn't as used anymore
- controllerDeadband: a decimal deadband, where the remaining area is scaled from 0-1 instead of deadband-1
- ___TranslationErrorMarginMeters: Two Constants to define "atGoal()" tolerance
- rotationLockErrorMargin determines atGoal() for rotationLock
- choreoXYThetaController are used for following Choreo auto paths
- auto & teleop DriveToPoint controllers are used for a more simple PID Alignment code
- Swerve Requests are necessary for the Pheonix Swerve API
- wantedState, systemState, desiredRotationForRotationLockState, desiredChoreoTrajectory, desiredPoseForDriveToPoint, etc. are the fields that hold setpoints
- IOInputs include all modules and one "swerveInputs"
- CommandXboxController is defined as controls are handled in the subsystem
- teleopVelocityCoefficient can be used to slow down the max speed for things like manual barging
## State-Based Subsystem
- handleStateTransitions(): switch statement on wantedState, determine the system state to be applied
- applyStates(): drive, choreo path, rotation lock, or drive to point are applied
## Usage
- drive.setWantedState(Drive.WantedState.TELEOP_DRIVE);
- drive.setTeleopVelocityCoefficient(defaultTeleopTranslationCoefficient);
- drive.setRotationVelocityCoefficient(1.0);
- If setting drive to point or rotation lock, call the expected method and make sure it changes wantedstate to desired
## Using as a template
- Generate the Pheonix Swerve API TunerConstants file
- Add the method:   
```java
@SuppressWarnings("unchecked")
  public static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      [] getModuleConstants() {
    return new SwerveModuleConstants[] {FrontLeft, FrontRight, BackLeft, BackRight};
  }
```
```java
drive = new Drive(new DriveIOCTRE(TunerConstants.getSwerveDrivetrainConstants(), TunerConstants.getModuleConstants()), 
controller, 
moduleConstants[0].SpeedAt12Volts, 
moduleConstants[0].SpeedAt12Volts / Math.hypot(moduleConstants[0].LocationX, moduleConstants[0].LocationY));
```