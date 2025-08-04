// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.archive.superstructure.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {
  // Hardware
  private final TalonFX talon;
  private final TalonFX followerTalon;

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerTorqueCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Temperature> followerTemp;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);
  private final Debouncer followerConnectedDebouncer = new Debouncer(0.5);

  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  public ElevatorIOTalonFX() {
    talon = new TalonFX(17);
    followerTalon = new TalonFX(16);
    followerTalon.setControl(new Follower(talon.getDeviceID(), true));

    // Configure motor
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    torqueCurrent = talon.getTorqueCurrent();
    supplyCurrent = talon.getSupplyCurrent();
    temp = talon.getDeviceTemp();
    followerAppliedVolts = followerTalon.getMotorVoltage();
    followerTorqueCurrent = followerTalon.getTorqueCurrent();
    followerSupplyCurrent = followerTalon.getSupplyCurrent();
    followerTemp = followerTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity,
        appliedVolts,
        supplyCurrent,
        temp,
        followerAppliedVolts,
        followerTorqueCurrent,
        followerSupplyCurrent,
        followerTemp);
    torqueCurrent.setUpdateFrequency(250);
    ParentDevice.optimizeBusUtilizationForAll(talon, followerTalon);

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        false,
        position,
        velocity,
        appliedVolts,
        torqueCurrent,
        supplyCurrent,
        temp,
        followerAppliedVolts,
        followerTorqueCurrent,
        followerSupplyCurrent,
        followerTemp);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.data =
        new ElevatorIOData(
            connectedDebouncer.calculate(
                // Exclude torque-current b/c it's running at a much higher update rate
                BaseStatusSignal.isAllGood(position, velocity, appliedVolts, supplyCurrent, temp)),
            followerConnectedDebouncer.calculate(
                BaseStatusSignal.isAllGood(
                    followerAppliedVolts,
                    followerTorqueCurrent,
                    followerSupplyCurrent,
                    followerTemp)),
            position.getValueAsDouble(),
            Units.rotationsToRadians(velocity.getValueAsDouble()),
            appliedVolts.getValueAsDouble(),
            torqueCurrent.getValueAsDouble(),
            supplyCurrent.getValueAsDouble(),
            temp.getValueAsDouble(),
            followerAppliedVolts.getValueAsDouble(),
            followerTorqueCurrent.getValueAsDouble(),
            followerSupplyCurrent.getValueAsDouble(),
            followerTemp.getValueAsDouble());
  }

  @Override
  public void runOpenLoop(double output) {
    talon.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  @Override
  public void runPosition(double positionRad, double feedforward) {
    talon.setControl(positionVoltageRequest.withPosition(positionRad).withFeedForward(feedforward));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () -> {
              config.MotorOutput.NeutralMode =
                  enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
              PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config));
              PhoenixUtil.tryUntilOk(5, () -> followerTalon.getConfigurator().apply(config));
            })
        .start();
  }
}
