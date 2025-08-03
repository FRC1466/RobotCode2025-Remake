// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.wrist;

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
import frc.robot.constants.WristConstants;
import frc.robot.util.TalonFXFactory;

public class WristIOTalonFX implements WristIO {
  private TalonFX wrist;

  DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
  MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withSlot(0);

  private final StatusSignal<Angle> wristPosition;
  private final StatusSignal<Voltage> wristVoltage;
  private final StatusSignal<Current> wristSupplyCurrent;
  private final StatusSignal<Current> wristStatorCurrent;
  private final StatusSignal<Temperature> wristTemperature;
  private final StatusSignal<AngularVelocity> wristAngularVelocity;
  private final StatusSignal<AngularAcceleration> wristAngularAcceleration;

  public WristIOTalonFX() {
    wrist = TalonFXFactory.createDefaultTalon(motorId);

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
        WristConstants.wristRadiansToRotations(accelerationConstraint);
    config.MotionMagic.MotionMagicCruiseVelocity =
        WristConstants.wristRadiansToRotations(velocityConstraint);
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    wrist.getConfigurator().apply(config);

    wristPosition = wrist.getRotorPosition();
    wristVoltage = wrist.getMotorVoltage();
    wristSupplyCurrent = wrist.getSupplyCurrent();
    wristStatorCurrent = wrist.getStatorCurrent();
    wristTemperature = wrist.getDeviceTemp();
    wristAngularVelocity = wrist.getRotorVelocity();
    wristAngularAcceleration = wrist.getAcceleration();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.wristAngle =
        Rotation2d.fromRadians(
            WristConstants.wristRotationsToRadians(wristPosition.getValueAsDouble()));

    inputs.wristAppliedVolts = wristVoltage.getValueAsDouble();
    inputs.wristSupplyCurrentAmps = wristSupplyCurrent.getValueAsDouble();
    inputs.wristStatorCurrentAmps = wristStatorCurrent.getValueAsDouble();
    inputs.wristMotorTemp = wristTemperature.getValueAsDouble();

    inputs.wristAngularVelocityRadPerSec =
        WristConstants.wristRotationsToRadians(wristAngularVelocity.getValueAsDouble());

    inputs.wristAngularAccelerationRadPerSecSquared =
        WristConstants.wristRotationsToRadians(wristAngularAcceleration.getValueAsDouble());
  }

  @Override
  public void setTargetAngle(Rotation2d target) {
    wrist.setControl(
        positionVoltage.withPosition(WristConstants.wristRadiansToRotations(target.getRadians())));
  }

  @Override
  public void resetWristAngle(Rotation2d angle) {
    wrist.setPosition(WristConstants.wristRadiansToRotations(angle.getRadians()));
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    wrist.setControl(dutyCycleOut.withOutput(dutyCycle));
  }

  @Override
  public void setNeutralMode(NeutralModeValue neutralMode) {
    wrist.setNeutralMode(neutralMode);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    var slot0Config = new Slot0Configs();
    slot0Config.kP = kP;
    slot0Config.kI = kI;
    slot0Config.kD = kD;
    wrist.getConfigurator().apply(slot0Config);
  }
}
