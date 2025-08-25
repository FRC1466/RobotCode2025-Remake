// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.coralSlapdown;

import static frc.robot.constants.CoralSlapdownConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulated implementation of the SlapdownIO interface for development and testing when not at the
 * lab. This class simulates the slapdown mechanism's behavior using the WPILib physics sim tools.
 */
public class CoralSlapdownIOSim implements CoralSlapdownIO {
  private static final double minAngleRads = 0.0;
  private static final double maxAngleRads = Math.PI * 2;
  private static final double armLength = 0.4;
  private static final double massKg = 4;

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          slapdownReduction,
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
          new TrapezoidProfile.Constraints(velocityConstraint, accelerationConstraint));
  private double appliedVoltage = 0.0;
  private boolean closedLoop = false;

  @Override
  public void updateInputs(CoralSlapdownIOInputs inputs) {
    sim.setInputVoltage(appliedVoltage);
    sim.update(0.02);

    double prevVelocity = inputs.slapdownAngularVelocityRadPerSec;

    inputs.slapdownAngle = Rotation2d.fromRadians(sim.getAngleRads());
    inputs.slapdownAngularVelocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.slapdownAngularAccelerationRadPerSecSquared =
        (inputs.slapdownAngularVelocityRadPerSec - prevVelocity) / 0.02;

    inputs.slapdownAppliedVolts = appliedVoltage;
    inputs.slapdownSupplyCurrentAmps = Math.abs(appliedVoltage) * 10.0;
    inputs.slapdownStatorCurrentAmps = inputs.slapdownSupplyCurrentAmps;
    inputs.slapdownMotorTemp = 40.0;

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
  public void resetSlapdownAngle(Rotation2d angle) {
    sim.setState(angle.getRadians(), 0.0);
    pid.reset(angle.getRadians());
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
