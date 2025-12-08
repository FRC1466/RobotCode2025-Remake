// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ElevatorSubsystem extends SubsystemBase {
  // Constants
  private static final int masterMotorCanId = 2;
  private static final int followerMotorCanId = 3;
  private static final double sprocketTeeth = 22;
  private static final double chainPitchInches = 0.25;

  // Motor controller
  private final TalonFX elevatorMasterMotor = new TalonFX(masterMotorCanId);
  private final TalonFX elevatorFollowerMotor = new TalonFX(followerMotorCanId);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withMechanismCircumference(
              Meters.of(Inches.of(chainPitchInches).in(Meters) * sprocketTeeth))
          .withClosedLoopController(
              1, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
          .withSimClosedLoopController(
              4, 0, 0, MetersPerSecond.of(3), MetersPerSecondPerSecond.of(9))
          .withSoftLimit(Meters.of(0), Meters.of(2.5))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withFeedforward(new ElevatorFeedforward(0, 0, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController master =
      new TalonFXWrapper(elevatorMasterMotor, DCMotor.getFalcon500(1), motorConfig);

  // Mechanism position config
  private final MechanismPositionConfig robotToMechanism =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5))
          .withMaxRobotLength(Meters.of(0.75))
          .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));

  // Elevator config
  private final ElevatorConfig elevatorConfig =
      new ElevatorConfig(master)
          .withStartingHeight(Meters.of(0))
          .withHardLimits(Meters.of(0), Meters.of(3))
          .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(robotToMechanism)
          .withMass(Pounds.of(16));

  private final Elevator elevator = new Elevator(elevatorConfig);

  public ElevatorSubsystem() {
    elevatorFollowerMotor.setControl(new Follower(masterMotorCanId, true));
  }

  @Override
  public void periodic() {
    elevator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    elevator.simIterate();
  }

  public Command setDutyCycle(double dutyCycle) {
    return elevator.set(dutyCycle);
  }

  public Command setHeight(Distance height) {
    return elevator.setHeight(height);
  }

  public Distance getHeight() {
    return elevator.getHeight();
  }

  public double getSetpointHeight() {
    return elevator.getMechanismSetpoint().orElse(Degrees.zero()).in(Degrees);
  }

  public Command runSysId() {
    return elevator.sysId(Volts.of(12), Volts.of(12).per(Second), Seconds.of(30));
  }
}
