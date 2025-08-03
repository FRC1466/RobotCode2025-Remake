// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.wrist;

import static frc.robot.constants.WristConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulated implementation of the WristIO interface for development and testing when not at the
 * lab. This class simulates the wrist's behavior using the WristSim class from WPILib. To learn
 * more on implementing WPILib simulations, view their documentation here:
 * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html
 */
public class WristIOSim implements WristIO {
  private static final double minAngleRads = 0.0;
  private static final double maxAngleRads = Math.PI * 2;
  private static final double armLength = 0.4;
  private static final double massKg = 4;

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          wristReduction,
          SingleJointedArmSim.estimateMOI(armLength, massKg),
          armLength,
          minAngleRads,
          maxAngleRads,
          false,
          0);

  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          kP.get(),
          kI.get(),
          kD.get(),
          new TrapezoidProfile.Constraints(
              velocityConstraint, // Max velocity (rad/s)
              accelerationConstraint // Max acceleration (rad/s^2)
              ));
  private double appliedVoltage = 0.0;
  private boolean closedLoop = false;

  @Override
  public void updateInputs(WristIOInputs inputs) {
    sim.setInputVoltage(appliedVoltage);
    sim.update(0.02);

    double prevVelocity = inputs.wristAngularVelocityRadPerSec;

    inputs.wristAngle = Rotation2d.fromRadians(sim.getAngleRads());
    inputs.wristAngularVelocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.wristAngularAccelerationRadPerSecSquared =
        (inputs.wristAngularVelocityRadPerSec - prevVelocity) / 0.02;

    inputs.wristAppliedVolts = appliedVoltage;
    inputs.wristSupplyCurrentAmps = Math.abs(appliedVoltage) * 10.0;
    inputs.wristStatorCurrentAmps = inputs.wristSupplyCurrentAmps;
    inputs.wristMotorTemp = 40.0;

    if (!DriverStation.isEnabled()) {
      appliedVoltage = 0.0;
    } else if (closedLoop) {
      appliedVoltage = pid.calculate(sim.getAngleRads());
      appliedVoltage = Math.max(-12.0, Math.min(12.0, appliedVoltage));
    }
  }

  @Override
  public void setTargetAngle(Rotation2d target) {
    closedLoop = true;
    pid.setGoal(Math.max(minAngleRads, Math.min(maxAngleRads, target.getRadians())));
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    closedLoop = false;
    appliedVoltage = dutyCycle * RobotController.getBatteryVoltage();
  }

  @Override
  public void resetWristAngle(Rotation2d angle) {
    sim.setState(angle.getRadians(), 0.0);
    pid.reset(angle.getRadians());
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
