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
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX elevatorMaster;
  private final TalonFX elevatorFollower;

  private final Follower followControlRequest;
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
  private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withSlot(0);

  private final StatusSignal<Angle> elevatorPositionMeters;
  private final StatusSignal<Voltage> elevatorAppliedVolts;
  private final StatusSignal<Current> elevatorSupplyCurrentAmps;
  private final StatusSignal<Current> elevatorStatorCurrentAmps;
  private final StatusSignal<AngularVelocity> elevatorVelocityMetersPerSec;
  private final StatusSignal<AngularAcceleration> elevatorAccelerationMetersPerSecSquared;
  private final StatusSignal<Temperature> elevatorMasterMotorTemp;
  private final StatusSignal<Temperature> elevatorFollowerMotorTemp;

  public ElevatorIOTalonFX() {
    elevatorMaster = new TalonFX(masterMotorId, "");
    elevatorFollower = new TalonFX(followerMotorId, "");
    followControlRequest = new Follower(masterMotorId, true);

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    talonConfig.CurrentLimits.StatorCurrentLimit = 120.0;
    talonConfig.Slot0.kP = kP.get();
    talonConfig.Slot0.kI = kI.get();
    talonConfig.Slot0.kD = kD.get();
    talonConfig.Slot0.kS = kS.get();

    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    talonConfig.MotionMagic.MotionMagicAcceleration = accelerationConstraint.get();
    talonConfig.MotionMagic.MotionMagicCruiseVelocity = velocityConstraint.get();
    talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    elevatorMaster.getConfigurator().apply(talonConfig);
    elevatorFollower.getConfigurator().apply(talonConfig);

    elevatorFollower.setControl(followControlRequest);

    elevatorPositionMeters = elevatorMaster.getPosition();
    elevatorAppliedVolts = elevatorMaster.getMotorVoltage();
    elevatorSupplyCurrentAmps = elevatorMaster.getSupplyCurrent();
    elevatorStatorCurrentAmps = elevatorMaster.getStatorCurrent();
    elevatorVelocityMetersPerSec = elevatorMaster.getRotorVelocity();
    elevatorAccelerationMetersPerSecSquared = elevatorMaster.getAcceleration();
    elevatorMasterMotorTemp = elevatorMaster.getDeviceTemp();
    elevatorFollowerMotorTemp = elevatorFollower.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        elevatorPositionMeters,
        elevatorAppliedVolts,
        elevatorSupplyCurrentAmps,
        elevatorStatorCurrentAmps,
        elevatorVelocityMetersPerSec,
        elevatorAccelerationMetersPerSecSquared,
        elevatorMasterMotorTemp,
        elevatorFollowerMotorTemp);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        elevatorPositionMeters,
        elevatorAppliedVolts,
        elevatorSupplyCurrentAmps,
        elevatorStatorCurrentAmps,
        elevatorVelocityMetersPerSec,
        elevatorAccelerationMetersPerSecSquared,
        elevatorMasterMotorTemp,
        elevatorFollowerMotorTemp);

    inputs.data =
        new ElevatorIOData(
            elevatorMaster.isAlive(),
            elevatorFollower.isAlive(),
            elevatorPositionMeters.getValueAsDouble(),
            elevatorVelocityMetersPerSec.getValueAsDouble(),
            elevatorAccelerationMetersPerSecSquared.getValueAsDouble(),
            elevatorAppliedVolts.getValueAsDouble(),
            elevatorSupplyCurrentAmps.getValueAsDouble(),
            elevatorStatorCurrentAmps.getValueAsDouble(),
            elevatorMasterMotorTemp.getValueAsDouble(),
            elevatorFollowerMotorTemp.getValueAsDouble());
  }

  @Override
  public void setTargetPosition(double positionInMeters) {
    elevatorMaster.setControl(positionVoltage.withPosition(positionInMeters));
  }

  @Override
  public void resetElevatorPosition(double positionInMeters) {
    elevatorMaster.setPosition(positionInMeters);
  }

  @Override
  public void setNeutralMode(NeutralModeValue neutralMode) {
    elevatorMaster.setNeutralMode(neutralMode);
    elevatorFollower.setNeutralMode(neutralMode);
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    elevatorMaster.setControl(dutyCycleOut.withOutput(dutyCycle));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    elevatorMaster.getConfigurator().apply(config);
  }
}
