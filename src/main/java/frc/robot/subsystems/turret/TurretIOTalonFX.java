// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import static frc.robot.constants.TurretConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.TurretConstants;
import frc.robot.util.TalonFXFactory;

public class TurretIOTalonFX implements TurretIO {
  private TalonFX turret;

  DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
  MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withSlot(0);

  private final StatusSignal<Angle> turretPosition;
  private final StatusSignal<Voltage> turretVoltage;
  private final StatusSignal<Current> turretSupplyCurrent;
  private final StatusSignal<Current> turretStatorCurrent;
  private final StatusSignal<Temperature> turretTemperature;
  private final StatusSignal<AngularVelocity> turretAngularVelocity;
  private final StatusSignal<AngularAcceleration> turretAngularAcceleration;

  public TurretIOTalonFX() {
    turret = TalonFXFactory.createDefaultTalon(motorId);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimit = 80.0;

    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();
    config.Slot0.kS = kS.get();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotionMagic.MotionMagicAcceleration =
        TurretConstants.turretRadiansToRotations(accelerationConstraint);
    config.MotionMagic.MotionMagicCruiseVelocity =
        TurretConstants.turretRadiansToRotations(velocityConstraint);
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    turret.getConfigurator().apply(config);

    turretPosition = turret.getRotorPosition();
    turretVoltage = turret.getMotorVoltage();
    turretSupplyCurrent = turret.getSupplyCurrent();
    turretStatorCurrent = turret.getStatorCurrent();
    turretTemperature = turret.getDeviceTemp();
    turretAngularVelocity = turret.getRotorVelocity();
    turretAngularAcceleration = turret.getAcceleration();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretAngle =
        Rotation2d.fromRadians(
            TurretConstants.turretRotationsToRadians(turretPosition.getValueAsDouble()));

    inputs.turretAppliedVolts = turretVoltage.getValueAsDouble();
    inputs.turretSupplyCurrentAmps = turretSupplyCurrent.getValueAsDouble();
    inputs.turretStatorCurrentAmps = turretStatorCurrent.getValueAsDouble();
    inputs.turretMotorTemp = turretTemperature.getValueAsDouble();

    inputs.turretAngularVelocityRadPerSec =
        TurretConstants.turretRotationsToRadians(turretAngularVelocity.getValueAsDouble());

    inputs.turretAngularAccelerationRadPerSecSquared =
        TurretConstants.turretRotationsToRadians(turretAngularAcceleration.getValueAsDouble());
  }

  @Override
  public void setTargetAngle(Rotation2d target) {
    turret.setControl(
        positionVoltage.withPosition(
            TurretConstants.turretRadiansToRotations(target.getRadians())));
  }

  @Override
  public void resetTurretAngle(Rotation2d angle) {
    turret.setPosition(TurretConstants.turretRadiansToRotations(angle.getRadians()));
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    turret.setControl(dutyCycleOut.withOutput(dutyCycle));
  }

  @Override
  public void setNeutralMode(NeutralModeValue neutralMode) {
    turret.setNeutralMode(neutralMode);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    var slot0Config = new Slot0Configs();
    slot0Config.kP = kP;
    slot0Config.kI = kI;
    slot0Config.kD = kD;
    turret.getConfigurator().apply(slot0Config);
  }
}
