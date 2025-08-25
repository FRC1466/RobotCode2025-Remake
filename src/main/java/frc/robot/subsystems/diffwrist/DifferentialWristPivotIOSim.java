// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.diffwrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Sim implementation for a differential wrist/pivot mechanism. This approximates each axis as an
 * independent single-jointed arm for simplicity.
 */
public class DifferentialWristPivotIOSim implements DifferentialWristPivotIO {
  // Simple approximations for both axes
  private static final DCMotor motor = DCMotor.getKrakenX60(1);
  private static final double pivotGearing = 100.0; // placeholder gearing
  private static final double wristGearing = 50.0; // placeholder gearing
  private static final double armLengthMeters = 0.3;
  private static final double minAngleRad = -Math.PI;
  private static final double maxAngleRad = Math.PI;
  private static final double moi = 0.05; // moment of inertia kg*m^2
  private static final double simDt = 0.02;

  private final SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          motor, pivotGearing, moi, armLengthMeters, minAngleRad, maxAngleRad, false, 0);
  private final SingleJointedArmSim wristSim =
      new SingleJointedArmSim(
          motor, wristGearing, moi, armLengthMeters, minAngleRad, maxAngleRad, false, 0);

  private final PIDController pivotPid = new PIDController(10.0, 0.0, 0.0);
  private final PIDController wristPid = new PIDController(10.0, 0.0, 0.0);

  private boolean pivotClosedLoop = false;
  private boolean wristClosedLoop = false;
  private double pivotTargetRad = 0.0;
  private double wristTargetRad = 0.0;

  private double pivotAppliedVolts = 0.0;
  private double wristAppliedVolts = 0.0;

  @Override
  public void updateInputs(DifferentialWristPivotIOInputs inputs) {
    // Apply voltages and advance sim
    pivotSim.setInputVoltage(pivotAppliedVolts);
    wristSim.setInputVoltage(wristAppliedVolts);
    pivotSim.update(simDt);
    wristSim.update(simDt);

    // Populate inputs
    double prevPivotVel = inputs.pivotAngularVelocityRadPerSec;
    double prevWristVel = inputs.wristAngularVelocityRadPerSec;

    inputs.pivotAngle = Rotation2d.fromRadians(pivotSim.getAngleRads());
    inputs.wristAngle = Rotation2d.fromRadians(wristSim.getAngleRads());
    inputs.pivotAngularVelocityRadPerSec = pivotSim.getVelocityRadPerSec();
    inputs.wristAngularVelocityRadPerSec = wristSim.getVelocityRadPerSec();
    inputs.pivotAngularAccelerationRadPerSecSquared =
        (inputs.pivotAngularVelocityRadPerSec - prevPivotVel) / simDt;
    inputs.wristAngularAccelerationRadPerSecSquared =
        (inputs.wristAngularVelocityRadPerSec - prevWristVel) / simDt;

    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.wristAppliedVolts = wristAppliedVolts;
    inputs.pivotSupplyCurrentAmps = Math.abs(pivotAppliedVolts) * 2.0;
    inputs.pivotStatorCurrentAmps = inputs.pivotSupplyCurrentAmps;
    inputs.wristSupplyCurrentAmps = Math.abs(wristAppliedVolts) * 2.0;
    inputs.wristStatorCurrentAmps = inputs.wristSupplyCurrentAmps;
    inputs.pivotMotorTempC = 40.0;
    inputs.wristMotorTempC = 40.0;

    // Control updates
    if (!DriverStation.isEnabled()) {
      pivotAppliedVolts = 0.0;
      wristAppliedVolts = 0.0;
    } else {
      if (pivotClosedLoop) {
        pivotAppliedVolts = pivotPid.calculate(pivotSim.getAngleRads(), pivotTargetRad);
        pivotAppliedVolts = Math.max(-12.0, Math.min(12.0, pivotAppliedVolts));
      }
      if (wristClosedLoop) {
        wristAppliedVolts = wristPid.calculate(wristSim.getAngleRads(), wristTargetRad);
        wristAppliedVolts = Math.max(-12.0, Math.min(12.0, wristAppliedVolts));
      }
    }
  }

  @Override
  public void setTargetAngles(Rotation2d pivotTarget, Rotation2d wristTarget) {
    pivotClosedLoop = true;
    wristClosedLoop = true;
    pivotTargetRad = clamp(pivotTarget.getRadians(), minAngleRad, maxAngleRad);
    wristTargetRad = clamp(wristTarget.getRadians(), minAngleRad, maxAngleRad);
  }

  @Override
  public void resetAngles(Rotation2d pivotAngle, Rotation2d wristAngle) {
    pivotSim.setState(pivotAngle.getRadians(), 0.0);
    wristSim.setState(wristAngle.getRadians(), 0.0);
  }

  @Override
  public void setDutyCycles(double pivotDuty, double wristDuty) {
    pivotClosedLoop = false;
    wristClosedLoop = false;
    double batt = RobotController.getBatteryVoltage();
    pivotAppliedVolts = clamp(pivotDuty, -1.0, 1.0) * batt;
    wristAppliedVolts = clamp(wristDuty, -1.0, 1.0) * batt;
  }

  @Override
  public void setPIDPivot(double kP, double kI, double kD) {
    pivotPid.setPID(kP, kI, kD);
  }

  @Override
  public void setPIDWrist(double kP, double kI, double kD) {
    wristPid.setPID(kP, kI, kD);
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
