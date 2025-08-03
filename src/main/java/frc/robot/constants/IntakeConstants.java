package frc.robot.constants;

public class IntakeConstants {
        // Coral operations
        public static final RollerVoltages coralIntake = new RollerVoltages(-1.25, -4);
        public static final RollerVoltages coralGrip = new RollerVoltages(0.2, 0);
        public static final RollerVoltages coralOuttake = new RollerVoltages(-10, 0);
        public static final RollerVoltages coralOuttakeL1 = new RollerVoltages(4, 0);
        public static final RollerVoltages coralBackup = new RollerVoltages(0.5, 0);

        // Algae operations
        public static final RollerVoltages algaeIntake = new RollerVoltages(3.0, 0);
        public static final RollerVoltages algaeHold = new RollerVoltages(0.4, 0);
        public static final RollerVoltages algaeEject = new RollerVoltages(-2.5, 0);

    public static final double topRollerCurrentThresholdForAlgaeDetection = 9.0;
    public static final double topRollerVelocityRpsThresholdForAlgaeDetectionAllowance = -80.0;
    public static final double topRollerVelocityRpsThresholdForAlgaeDetectionWhileIntaking = -20.0;
    public static final double topRollerVelocityRpsThresholdForAlgaeDetectionWhileHolding = -70.0;

    public static final class RollerVoltages {
        public final double endEffectorVoltage;
        public final double starWheelVoltage;
        
        public RollerVoltages(double endEffectorVoltage, double starWheelVoltage) {
            this.endEffectorVoltage = endEffectorVoltage;
            this.starWheelVoltage = starWheelVoltage;
        }
    }
}
