This document explains the `Vision.java` subsystem. This class is responsible for processing data from one or more vision cameras to determine the robot's position on the field. Its main responsibilities are:

*   Interfacing with multiple cameras using the AdvantageKit hardware abstraction (`IO`) layer.
*   Receiving pose estimations (robot position and rotation) from each camera.
*   Filtering out unreliable or unrealistic pose estimations.
*   Calculating the confidence (standard deviation) of each valid measurement.
*   Passing valid, timestamped pose measurements to a "consumer," which is typically the `Drive` subsystem's pose estimator.

### Core Components (Member Variables)

These are the main objects the `Vision` subsystem uses to function.

*   **`consumer`**: A `VisionConsumer` functional interface. This is a key part of the subsystem's design. The `Vision` class doesn't directly know about the `Drive` subsystem; it just knows it needs to send its data to a consumer. This makes the code modular and testable. In practice, the consumer is the `drive::addVisionMeasurement` method.
*   **`io`**: An array of `VisionIO` objects, one for each physical camera. This follows the AdvantageKit pattern of separating hardware-specific code (`IO`) from the subsystem logic.
*   **`inputs`**: An array of `VisionIOInputsAutoLogged` objects. Each `inputs` object stores the latest data read from its corresponding `VisionIO` interface, such as connection status, tag IDs, and pose observations.
*   **`disconnectedAlerts`**: An array of `Alert` objects used to warn the driver if a camera loses connection.

### Method Breakdown

#### `Vision(...)` (The Constructor)

This is the initialization code that runs once when the robot starts.

*   **Dependency Injection**: The constructor takes a `VisionConsumer` and a variable number of `VisionIO` objects as arguments. This allows the same `Vision` subsystem code to be used for different camera hardware (e.g., `VisionIOPhotonVision`, `VisionIOLimelight`) and for simulation (`VisionIOSim`) by "injecting" the appropriate implementations.
*   **Initialization**: It creates the `inputs` and `disconnectedAlerts` arrays, with one entry for each `VisionIO` object provided.

#### `periodic()`

This method is called in a loop, approximately 50 times per second. This is where the core logic happens.

1.  **Update Inputs**: It loops through each camera's `io` interface and calls `updateInputs()` to get the latest data. It then logs this raw data using `Logger.processInputs()`.
2.  **Check Connection**: It updates the `disconnectedAlerts` for each camera based on its connection status.
3.  **Process Observations**: For each camera, it iterates through the list of `poseObservations` received in the latest update.
4.  **Filtering**: For each observation, it performs a series of checks to decide if the pose is valid (`rejectPose = false`) or should be ignored (`rejectPose = true`). A pose is rejected if:
    *   It was generated from zero tags.
    *   It was generated from a single tag and has high ambiguity.
    *   The calculated height of the robot (`pose.getZ()`) is unrealistic.
    *   The calculated position (`pose.getX()`, `pose.getY()`) is outside the known field dimensions.
5.  **Calculate Standard Deviation**: If a pose is accepted, the code calculates a standard deviation for the measurement. This value represents the confidence in the pose. The confidence decreases (standard deviation increases) as the average distance to the tags increases. This value is crucial for the `poseEstimator` in the `Drive` subsystem, as it tells it how much to trust this vision measurement compared to its existing odometry data.
6.  **Send to Consumer**: The accepted pose, its timestamp, and its calculated standard deviations are passed to the `consumer.accept(...)` method. This sends the data to the `Drive` subsystem's `addVisionMeasurement` method, which fuses it into the robot's main pose estimate.
7.  **Log for Debugging**: The subsystem logs all poses, categorizing them as "Accepted" or "Rejected". This is extremely useful for tuning and debugging vision performance using AdvantageScope.

#### `getTargetX(int cameraIndex)`

*   A simple helper method that returns the horizontal angle (`tx`) to the primary target seen by a specific camera. This can be used for basic, non-pose-based alignment tasks (e.g., "point the robot at the speaker" in 2024).

#### `VisionConsumer` (Functional Interface)

*   This is a simple interface defined inside the `Vision` class. It specifies a single method, `accept`. By using this interface, the `Vision` subsystem is decoupled from the `Drive` subsystem, adhering to good software design principles.
