package frc.robot.constants;

public class ElevatorConstants {
    // Motor IDs
    public static final int masterMotorId = 17;
    public static final int followerMotorId = 16;

    // Current Limits
    public static final double supplyCurrentLimit = 60.0;
    public static final double statorCurrentLimit = 120.0;

    // PID Constants
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kG = 0.0;
    public static final double kS = 0.0;

    // Motion Magic Constants
    public static final double accelerationConstraint = 8;
    public static final double accelerationConstraintAlgae = 6;
    public static final double accelerationConstraintDown = 6;

    public static final double velocityConstraint = 3;
    public static final double velocityConstraintAlgae = 2.8;
}
