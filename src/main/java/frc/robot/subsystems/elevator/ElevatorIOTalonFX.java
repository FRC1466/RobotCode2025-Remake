// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.constants.ElevatorConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator.ElevatorProfile;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.TalonFXFactory;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX masterTalonFX;
  private final TalonFX followerTalonFX;
  Follower followControlRequest;
  DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
  MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withSlot(0);

  private final StatusSignal<Angle> positionInRotations;
  private final StatusSignal<AngularVelocity> velocityRotationsPerSec;
  private final StatusSignal<AngularAcceleration> accelerationRotationsPerSecSquared;
  private final StatusSignal<Voltage> masterAppliedVolts;
  private final StatusSignal<Current> masterStatorCurrentAmps;
  private final StatusSignal<Current> masterSupplyCurrentAmps;
  private final StatusSignal<Temperature> masterTemp;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerStatorCurrentAmps;
  private final StatusSignal<Current> followerSupplyCurrentAmps;
  private final StatusSignal<Temperature> followerTemp;

  public ElevatorIOTalonFX() {
    masterTalonFX = TalonFXFactory.createDefaultTalon(masterMotorId);
    followerTalonFX = TalonFXFactory.createDefaultTalon(followerMotorId);
    followControlRequest = new Follower(masterMotorId, false);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimit = 120.0;

    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();
    config.Slot0.kS = kS.get();
    config.Slot0.kG = kG.get();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotionMagic.MotionMagicAcceleration =
        ElevatorConstants.elevatorMetersToRotations(ElevatorProfile.DEFAULT.acceleration.get());
    config.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.elevatorMetersToRotations(ElevatorProfile.DEFAULT.velocity.get());

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    PhoenixUtil.tryUntilOk(5, () -> masterTalonFX.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> followerTalonFX.getConfigurator().apply(config));

    followerTalonFX.setControl(followControlRequest);

    positionInRotations = masterTalonFX.getPosition();
    velocityRotationsPerSec = masterTalonFX.getRotorVelocity();
    accelerationRotationsPerSecSquared = masterTalonFX.getAcceleration();
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
        positionInRotations,
        velocityRotationsPerSec,
        accelerationRotationsPerSecSquared,
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
    inputs.elevatorPositionMeters =
        ElevatorConstants.elevatorRotationsToMeters(positionInRotations.getValueAsDouble());
    inputs.elevatorVelocityMetersPerSec =
        ElevatorConstants.elevatorRotationsToMeters(velocityRotationsPerSec.getValueAsDouble());
    inputs.elevatorAccelerationMetersPerSecSquared =
        ElevatorConstants.elevatorRotationsToMeters(
            accelerationRotationsPerSecSquared.getValueAsDouble());

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
            ElevatorConstants.elevatorMetersToRotations(positionInMeters)));
  }

  @Override
  public void resetElevatorPosition(double positionInMeters) {
    masterTalonFX.setPosition(ElevatorConstants.elevatorMetersToRotations(positionInMeters));
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

  @Override
  public void setMotionProfileConstraints(ElevatorProfile elevatorProfile) {
    masterTalonFX
        .getConfigurator()
        .apply(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(
                    ElevatorConstants.elevatorMetersToRotations(elevatorProfile.velocity.get()))
                .withMotionMagicAcceleration(
                    ElevatorConstants.elevatorMetersToRotations(
                        elevatorProfile.acceleration.get())));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    var slot0Config = new Slot0Configs();
    slot0Config.kP = kP;
    slot0Config.kI = kI;
    slot0Config.kD = kD;
    masterTalonFX.getConfigurator().apply(slot0Config);
  }
}
