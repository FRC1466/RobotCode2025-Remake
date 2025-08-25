// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.algaeSlapdown;

import static frc.robot.constants.AlgaeSlapdownConstants.*;

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
import frc.robot.constants.AlgaeSlapdownConstants;
import frc.robot.util.TalonFXFactory;

public class AlgaeSlapdownIOTalonFX implements AlgaeSlapdownIO {
  private final TalonFX slapdown;

  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
  private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withSlot(0);

  private final StatusSignal<Angle> rotorPosition;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> deviceTemp;
  private final StatusSignal<AngularVelocity> rotorVelocity;
  private final StatusSignal<AngularAcceleration> rotorAcceleration;

  public AlgaeSlapdownIOTalonFX() {
    slapdown = TalonFXFactory.createDefaultTalon(motorId);

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    cfg.CurrentLimits.StatorCurrentLimit = 80.0;

    cfg.Slot0.kP = kP.get();
    cfg.Slot0.kI = kI.get();
    cfg.Slot0.kD = kD.get();
    cfg.Slot0.kS = kS.get();

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotionMagic.MotionMagicAcceleration =
        AlgaeSlapdownConstants.wristRadiansToRotations(accelerationConstraint);
    cfg.MotionMagic.MotionMagicCruiseVelocity =
        AlgaeSlapdownConstants.wristRadiansToRotations(velocityConstraint);
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    slapdown.getConfigurator().apply(cfg);

    rotorPosition = slapdown.getRotorPosition();
    motorVoltage = slapdown.getMotorVoltage();
    supplyCurrent = slapdown.getSupplyCurrent();
    statorCurrent = slapdown.getStatorCurrent();
    deviceTemp = slapdown.getDeviceTemp();
    rotorVelocity = slapdown.getRotorVelocity();
    rotorAcceleration = slapdown.getAcceleration();
  }

  @Override
  public void updateInputs(AlgaeSlapdownIOInputs inputs) {
    inputs.slapdownAngle =
        Rotation2d.fromRadians(
            AlgaeSlapdownConstants.wristRotationsToRadians(rotorPosition.getValueAsDouble()));
    inputs.slapdownAppliedVolts = motorVoltage.getValueAsDouble();
    inputs.slapdownSupplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.slapdownStatorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.slapdownMotorTemp = deviceTemp.getValueAsDouble();
    inputs.slapdownAngularVelocityRadPerSec =
        AlgaeSlapdownConstants.wristRotationsToRadians(rotorVelocity.getValueAsDouble());
    inputs.slapdownAngularAccelerationRadPerSecSquared =
        AlgaeSlapdownConstants.wristRotationsToRadians(rotorAcceleration.getValueAsDouble());
  }

  @Override
  public void setTargetAngle(Rotation2d target) {
    slapdown.setControl(
        positionVoltage.withPosition(
            AlgaeSlapdownConstants.wristRadiansToRotations(target.getRadians())));
  }

  @Override
  public void resetSlapdownAngle(Rotation2d angle) {
    slapdown.setPosition(AlgaeSlapdownConstants.wristRadiansToRotations(angle.getRadians()));
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    slapdown.setControl(dutyCycleOut.withOutput(dutyCycle));
  }

  @Override
  public void setNeutralMode(NeutralModeValue neutralMode) {
    slapdown.setNeutralMode(neutralMode);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slapdown.getConfigurator().apply(slot0);
  }
}
