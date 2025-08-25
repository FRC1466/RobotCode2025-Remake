// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import static frc.robot.constants.TurretConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulated implementation of the TurretIO interface for development and testing when not at the
 * lab. This class simulates the turret's behavior using the TurretSim class from WPILib. To learn
 * more on implementing WPILib simulations, view their documentation here:
 * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html
 */
public class TurretIOSim implements TurretIO {
  private static final double minAngleDegrees = -540;
  private static final double maxAngleDegrees = 540;
  private static final double armLength = 0.4;
  private static final double massKg = 4;

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          turretReduction,
          SingleJointedArmSim.estimateMOI(armLength, massKg),
          armLength,
          Math.toRadians(minAngleDegrees),
          Math.toRadians(maxAngleDegrees),
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
  public void updateInputs(TurretIOInputs inputs) {
    sim.setInputVoltage(appliedVoltage);
    sim.update(0.02);

    double prevVelocity = inputs.turretAngularVelocityRadPerSec;

    inputs.turretAngle = Rotation2d.fromRadians(sim.getAngleRads());
    inputs.turretAngularVelocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.turretAngularAccelerationRadPerSecSquared =
        (inputs.turretAngularVelocityRadPerSec - prevVelocity) / 0.02;

    inputs.turretAppliedVolts = appliedVoltage;
    inputs.turretSupplyCurrentAmps = Math.abs(appliedVoltage) * 10.0;
    inputs.turretStatorCurrentAmps = inputs.turretSupplyCurrentAmps;
    inputs.turretMotorTemp = 40.0;

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
    pid.setGoal(
        Math.max(
            Math.toRadians(minAngleDegrees),
            Math.min(Math.toRadians(maxAngleDegrees), target.getRadians())));
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    closedLoop = false;
    appliedVoltage = dutyCycle * RobotController.getBatteryVoltage();
  }

  @Override
  public void resetTurretAngle(Rotation2d angle) {
    sim.setState(angle.getRadians(), 0.0);
    pid.reset(angle.getRadians());
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
