// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.constants.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.elevator.Elevator.ElevatorProfile;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.TalonFXFactory;
import java.util.function.Supplier;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX masterTalonFX;
  private final TalonFX followerTalonFX;
  Follower followControlRequest;
  DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
  MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withSlot(0);

  private final StatusSignal<Angle> positionInMeters;
  private final StatusSignal<AngularVelocity> velocityMetersPerSec;
  private final StatusSignal<AngularAcceleration> accelerationMetersPerSecSquared;
  private final StatusSignal<Voltage> masterAppliedVolts;
  private final StatusSignal<Current> masterStatorCurrentAmps;
  private final StatusSignal<Current> masterSupplyCurrentAmps;
  private final StatusSignal<Temperature> masterTemp;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerStatorCurrentAmps;
  private final StatusSignal<Current> followerSupplyCurrentAmps;
  private final StatusSignal<Temperature> followerTemp;

  private Supplier<Rotation2d> shoulderAngleSupplier = () -> Rotation2d.kZero;

  public ElevatorIOTalonFX() {
    masterTalonFX = TalonFXFactory.createDefaultTalon(masterMotorId);
    followerTalonFX = TalonFXFactory.createDefaultTalon(followerMotorId);
    followControlRequest = new Follower(masterMotorId, false);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimit = 120.0;

    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kG = kG;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotionMagic.MotionMagicAcceleration = ElevatorProfile.DEFAULT.acceleration;
    config.MotionMagic.MotionMagicCruiseVelocity = ElevatorProfile.DEFAULT.velocity;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    masterTalonFX.getConfigurator().apply(config);
    followerTalonFX.getConfigurator().apply(config);

    followerTalonFX.setControl(followControlRequest);

    positionInMeters = masterTalonFX.getPosition();
    velocityMetersPerSec = masterTalonFX.getRotorVelocity();
    accelerationMetersPerSecSquared = masterTalonFX.getAcceleration();
    masterAppliedVolts = masterTalonFX.getMotorVoltage();
    masterStatorCurrentAmps = masterTalonFX.getStatorCurrent();
    masterSupplyCurrentAmps = masterTalonFX.getSupplyCurrent();
    masterTemp = masterTalonFX.getDeviceTemp();
    followerAppliedVolts = followerTalonFX.getMotorVoltage();
    followerStatorCurrentAmps = followerTalonFX.getStatorCurrent();
    followerSupplyCurrentAmps = followerTalonFX.getSupplyCurrent();
    followerTemp = followerTalonFX.getDeviceTemp();

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        false,
        positionInMeters,
        velocityMetersPerSec,
        masterAppliedVolts,
        masterStatorCurrentAmps,
        masterSupplyCurrentAmps,
        masterTemp,
        followerAppliedVolts,
        followerStatorCurrentAmps,
        followerSupplyCurrentAmps,
        followerTemp);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorPositionMeters = positionInMeters.getValueAsDouble() * EXTENSION_POSITION_COEFFICIENT;
    inputs.elevatorVelocityMetersPerSec = velocityMetersPerSec.getValueAsDouble() * EXTENSION_POSITION_COEFFICIENT;
    inputs.elevatorAccelerationMetersPerSecSquared = accelerationMetersPerSecSquared.getValueAsDouble() * EXTENSION_POSITION_COEFFICIENT;
    
    inputs.elevatorAppliedVolts = masterAppliedVolts.getValueAsDouble();
    inputs.elevatorSupplyCurrentAmps = masterSupplyCurrentAmps.getValueAsDouble();
    inputs.elevatorStatorCurrentAmps = masterStatorCurrentAmps.getValueAsDouble();
    
    inputs.elevatorMasterMotorTemp = masterTemp.getValueAsDouble();
    inputs.elevatorFollowerMotorTemp = followerTemp.getValueAsDouble();
  }

  @Override
  public void setTargetPosition(double positionInMeters) {
    masterTalonFX.setControl(
        positionVoltage.withPosition(
            (shoulderAngleSupplier.get().getRotations()
                    * SHOULDER_EXTENSION_COUPLING_RATIO)
                + positionInMeters / EXTENSION_POSITION_COEFFICIENT));
  }

  @Override
  public void resetElevatorPosition(double positionInMeters) {
    masterTalonFX.setPosition(positionInMeters / EXTENSION_POSITION_COEFFICIENT);
  }

  @Override
  public void setNeutralMode(NeutralModeValue neutralMode) {
    masterTalonFX.setNeutralMode(neutralMode);
    followerTalonFX.setNeutralMode(neutralMode);
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    masterTalonFX.setControl(dutyCycleOut.withOutput(dutyCycle));
  }
}
