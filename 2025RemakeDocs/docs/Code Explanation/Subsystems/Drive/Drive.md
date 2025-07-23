This document explains the `Drive.java` subsystem. This class is the central controller for the robot's swerve drive. It is responsible for:

*   Controlling the state (angle and velocity) of each swerve module.
*   Maintaining the robot's odometry (its estimated position on the field).
*   Interfacing with PathPlanner for autonomous path following.

### Constants

The `final` variables at the top of the file define the robot's physical properties and configuration. These values must be tuned for your specific robot.

*   **`ODOMETRY_FREQUENCY`**: The update rate for the odometry calculations. A higher frequency can be used with CAN FD.
*   **`DRIVE_BASE_LENGTH`**, **`DRIVE_BASE_WIDTH`**, **`DRIVE_BASE_RADIUS`**: The physical dimensions of the robot's wheelbase. Accurate measurements are critical for correct swerve kinematics.
*   **`robotWidth`**: The full width of the robot, including bumpers. This is used by PathPlanner for collision avoidance.
*   **PathPlanner Constants (`ROBOT_MASS_KG`, `ROBOT_MOI`, etc.)**: These values describe the robot's physical properties (mass, moment of inertia). PathPlanner uses them to generate physically accurate paths.

### Core Components (Member Variables)

These are the main objects the `Drive` subsystem uses to function.

*   **`gyroIO` & `gyroInputs`**: An abstraction for the gyroscope. This follows the AdvantageKit pattern of separating the hardware implementation (`IO`) from its data (`Inputs`), which makes the code easier to test and simulate.
*   **`modules`**: An array holding the four `Module` objects, one for each swerve module.
*   **`kinematics`**: A WPILib object that handles the mathematics of the swerve drive. It translates robot-level speeds (`ChassisSpeeds`) into individual module speeds and angles (`SwerveModuleState`), and vice-versa.
*   **`poseEstimator`**: A WPILib class that estimates the robot's position (`Pose2d`) on the field. It fuses data from the wheel encoders and the gyroscope to produce an accurate position estimate. It can also incorporate data from vision systems to correct for drift.
*   **`odometryLock`**: A lock to prevent data corruption. Since odometry may be updated on a separate, higher-frequency thread, this lock ensures that reading and writing position data do not happen simultaneously.

### Method Breakdown

#### `Drive(...)` (The Constructor)

This is the initialization code that runs once when the robot is enabled.

*   **Dependency Injection**: The constructor takes `GyroIO` and `ModuleIO` as arguments. This pattern allows the same `Drive` subsystem code to be used for both the real robot and a simulator by "injecting" the appropriate hardware implementation.
*   **Module Initialization**: It creates the four `Module` objects.
*   **PathPlanner Setup (`AutoBuilder.configure`)**: This block configures the interface between PathPlanner and the `Drive` subsystem. It provides PathPlanner with methods to:
    *   Get the robot's current position (`this::getPose`).
    *   Reset the robot's position (`this::setPose`).
    *   Get the robot's current velocity (`this::getChassisSpeeds`).
    *   Send drive commands to the robot (`this::runVelocity`). This is the core callback PathPlanner uses to execute a path.
    *   It also provides logic to flip paths based on the current alliance color.

#### `periodic()`

This method is called in a loop, approximately 50 times per second.

1.  **Lock**: It acquires the `odometryLock` to ensure thread-safe access to odometry data.
2.  **Update Inputs**: It calls `gyroIO.updateInputs()` and `module.periodic()` for all modules to get the latest sensor data.
3.  **Odometry Update**: It gets new, timestamped position data from the modules. For each data point, it updates the `poseEstimator` with the wheel positions and the corresponding gyroscope angle. This fusion of data maintains an accurate field-relative position estimate.
4.  **Unlock**: It releases the `odometryLock`.

#### `runVelocity(ChassisSpeeds speeds)`

This is the primary method for controlling the robot's movement.

*   It accepts a `ChassisSpeeds` object, which specifies the desired forward (`vx`), sideways (`vy`), and rotational (`omega`) speeds.
*   It uses `kinematics.toSwerveModuleStates()` to convert these robot-centric speeds into individual setpoints (angle and speed) for each module.
*   It calls `SwerveDriveKinematics.desaturateWheelSpeeds()` to scale down the module speeds if any of them exceed the physical maximum.
*   Finally, it sends these setpoints to each `Module` via `module.runSetpoint()`.

#### `stop()` and `stopWithX()`

*   **`stop()`**: A convenience method that calls `runVelocity` with zero speeds to halt the robot.
*   **`stopWithX()`**: Stops the robot and turns the wheels inward to form an "X" shape, making the robot more resistant to being pushed.

#### `get...()` Methods

These methods provide other parts of the code with information about the drivetrain's state.

*   **`getPose()`**: Returns the robot's current estimated position and rotation (`Pose2d`). This is essential for autonomous control.
*   **`getRotation()`**: Returns just the robot's rotation.
*   **`getChassisSpeeds()`**: Returns the robot's current measured velocity by converting module states back into robot-centric speeds.
*   **`getModuleStates()` / `getModulePositions()`**: Return the raw state (speed and angle) or position (distance and angle) of each module, which is useful for logging and debugging.

#### `setPose(Pose2d pose)`

This method manually overrides the robot's odometry. It is typically used at the start of a match to set the robot's initial position or to re-calibrate its position using vision data. It works by calling `poseEstimator.resetPosition()`.

#### `addVisionMeasurement(...)`

This method integrates position data from a vision system (like a Limelight).

*   The `poseEstimator` fuses the vision measurement with the existing gyro and encoder data.
*   The method requires a standard deviation matrix, which tells the estimator how much to trust the vision measurement. A low standard deviation indicates high confidence, causing the estimator to adjust its pose more significantly. This is key to correcting odometry drift during a match.
