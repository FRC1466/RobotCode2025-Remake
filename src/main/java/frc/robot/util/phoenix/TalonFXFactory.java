// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.phoenix;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import frc.robot.util.CanDevice;

/**
 * Creates FXTalons objects and configures all the parameters we care about to factory defaults.
 * Closed-loop and sensor parameters are not set, as these are expected to be set by the
 * application.
 */
public class TalonFXFactory {

  public static NeutralModeValue neautralMode = NeutralModeValue.Coast;
  public static InvertedValue invertValue = InvertedValue.CounterClockwise_Positive;
  public static double neautralDeadband = 0.04;

  // create a TalonFX with the default (out of the box) configuration
  public static TalonFX createDefaultTalon(CanDevice id) {
    return createDefaultTalon(id, true);
  }

  public static TalonFX createDefaultTalon(CanDevice id, boolean trigger_config) {
    var talon = createTalon(id);
    if (trigger_config) {
      PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(getDefaultConfig()));
    }
    return talon;
  }

  public static TalonFX createPermanentFollowerTalon(
      CanDevice followerId, CanDevice masterId, boolean opposeMasterDirection) {
    if (!followerId.getBus().equals(masterId.getBus())) {
      throw new RuntimeException("Master and Follower TalonFXs must be on the same CAN bus.");
    }
    final TalonFX talon = createTalon(followerId);
    talon.setControl(new Follower(masterId.getDeviceNumber(), opposeMasterDirection));
    return talon;
  }

  public static TalonFXConfiguration getDefaultConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = neautralMode;
    config.MotorOutput.Inverted = invertValue;
    config.MotorOutput.DutyCycleNeutralDeadband = neautralDeadband;
    config.MotorOutput.PeakForwardDutyCycle = 1.0;
    config.MotorOutput.PeakReverseDutyCycle = -1.0;

    config.CurrentLimits.SupplyCurrentLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimitEnable = false;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.FeedbackRotorOffset = 0;
    config.Feedback.SensorToMechanismRatio = 1;

    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
    config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    config.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
    config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

    config.Audio.BeepOnBoot = true;

    return config;
  }

  private static TalonFX createTalon(CanDevice id) {
    TalonFX talon = new TalonFX(id.getDeviceNumber(), id.getBus());
    talon.clearStickyFaults();

    return talon;
  }
}
