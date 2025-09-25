// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivot;

import static frc.robot.constants.WristConstants.*;

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
import frc.robot.util.TalonFXFactory;

public class PivotIOTalonFX implements PivotIO {
  private TalonFX pivot;

  DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
  MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withSlot(0);

  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<Voltage> pivotVoltage;
  private final StatusSignal<Current> pivotSupplyCurrent;
  private final StatusSignal<Current> pivotStatorCurrent;
  private final StatusSignal<Temperature> pivotTemperature;
  private final StatusSignal<AngularVelocity> pivotAngularVelocity;
  private final StatusSignal<AngularAcceleration> pivotAngularAcceleration;

  public PivotIOTalonFX() {
    pivot = TalonFXFactory.createDefaultTalon(motorId);

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
        wristRadiansToRotations(accelerationConstraint);
    config.MotionMagic.MotionMagicCruiseVelocity =
        wristRadiansToRotations(velocityConstraint);
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivot.getConfigurator().apply(config);

    pivotPosition = pivot.getRotorPosition();
    pivotVoltage = pivot.getMotorVoltage();
    pivotSupplyCurrent = pivot.getSupplyCurrent();
    pivotStatorCurrent = pivot.getStatorCurrent();
    pivotTemperature = pivot.getDeviceTemp();
    pivotAngularVelocity = pivot.getRotorVelocity();
    pivotAngularAcceleration = pivot.getAcceleration();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotAngle =
        Rotation2d.fromRadians(
            wristRotationsToRadians(pivotPosition.getValueAsDouble()));

    inputs.pivotAppliedVolts = pivotVoltage.getValueAsDouble();
    inputs.pivotSupplyCurrentAmps = pivotSupplyCurrent.getValueAsDouble();
    inputs.pivotStatorCurrentAmps = pivotStatorCurrent.getValueAsDouble();
    inputs.pivotMotorTemp = pivotTemperature.getValueAsDouble();

    inputs.pivotAngularVelocityRadPerSec =
        wristRotationsToRadians(pivotAngularVelocity.getValueAsDouble());

    inputs.pivotAngularAccelerationRadPerSecSquared =
        wristRotationsToRadians(pivotAngularAcceleration.getValueAsDouble());
  }

  @Override
  public void setTargetAngle(Rotation2d target) {
    pivot.setControl(
        positionVoltage.withPosition(wristRadiansToRotations(target.getRadians())));
  }

  @Override
  public void resetPivotAngle(Rotation2d angle) {
    pivot.setPosition(wristRadiansToRotations(angle.getRadians()));
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    pivot.setControl(dutyCycleOut.withOutput(dutyCycle));
  }

  @Override
  public void setNeutralMode(NeutralModeValue neutralMode) {
    pivot.setNeutralMode(neutralMode);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    var slot0Config = new Slot0Configs();
    slot0Config.kP = kP;
    slot0Config.kI = kI;
    slot0Config.kD = kD;
    pivot.getConfigurator().apply(slot0Config);
  }
}
