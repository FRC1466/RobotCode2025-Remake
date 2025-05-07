// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.manipulator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.util.PhoenixUtil;

public class PivotIOTalonFX implements PivotIO {
  private static final int encoderId = 0;

  // Hardware
  private final TalonFX talon;
  private final DutyCycleEncoder encoder;

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  public static final double absolutePositionOffset = -0.4601;

  // Status Signals
  private final StatusSignal<Angle> internalPosition;
  private final double encoderAbsolutePosition;
  private final StatusSignal<AngularVelocity> internalVelocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> temp;

  // Control Requests
  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOC =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionVoltage positionVoltage = new PositionVoltage(0).withUpdateFreqHz(0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  // Connected debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

  public PivotIOTalonFX() {
    talon = new TalonFX(14);
    encoder = new DutyCycleEncoder(encoderId);

    encoder.setDutyCycleRange(0, 1);

    // Configure motor
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    // Get and set status signals
    internalPosition = talon.getPosition();
    internalVelocity = talon.getVelocity();
    encoderAbsolutePosition = encoder.get();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrentAmps = talon.getSupplyCurrent();
    torqueCurrentAmps = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, internalVelocity, appliedVolts, supplyCurrentAmps, torqueCurrentAmps, temp);
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, internalPosition);
    talon.optimizeBusUtilization();

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        false,
        internalPosition,
        internalVelocity,
        appliedVolts,
        supplyCurrentAmps,
        torqueCurrentAmps,
        temp);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.data =
        new PivotIOData(
            motorConnectedDebouncer.calculate(
                BaseStatusSignal.isAllGood(
                    internalPosition,
                    internalVelocity,
                    appliedVolts,
                    supplyCurrentAmps,
                    torqueCurrentAmps,
                    temp)),
            true,
            Rotation2d.fromRotations(encoderAbsolutePosition)
                .minus(Rotation2d.fromRotations(absolutePositionOffset))
                .unaryMinus(),
            internalVelocity.getValue().in(RadiansPerSecond),
            appliedVolts.getValue().in(Volts),
            supplyCurrentAmps.getValue().in(Amps),
            torqueCurrentAmps.getValue().in(Amps),
            temp.getValue().in(Celsius));
  }

  @Override
  public void runOpenLoop(double output) {
    talon.setControl(torqueCurrentFOC.withOutput(output));
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
  public void runPosition(Rotation2d position, double feedforward) {
    talon.setControl(
        positionVoltage.withPosition(position.getRotations()).withFeedForward(feedforward));
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
              tryUntilOk(5, () -> talon.getConfigurator().apply(config));
            })
        .start();
  }
}
