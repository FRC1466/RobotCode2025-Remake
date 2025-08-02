// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

/**
 * Creates and configures TalonFX motor controllers.
 *
 * <p>This factory provides methods to create TalonFX instances with a default configuration. The
 * default configuration is suitable for general use, but application-specific settings like
 * closed-loop control parameters should be configured separately.
 */
public final class TalonFXFactory {

  private static final NeutralModeValue defaultNeutralMode = NeutralModeValue.Brake;
  private static final InvertedValue defaultInvertValue = InvertedValue.CounterClockwise_Positive;
  private static final double defaultNeutralDeadband = 0.04;
  private static final int configTimeoutRetries = 5;

  private TalonFXFactory() {
    // Prevent instantiation of this utility class.
  }

  /**
   * Creates a TalonFX with the default configuration.
   *
   * @param deviceId The CAN device ID.
   * @param canBus The CAN bus name ("rio", "canivore", etc.).
   * @return A configured TalonFX instance.
   */
  public static TalonFX createDefaultTalon(int deviceId) {
    return createDefaultTalon(deviceId, "rio", true);
  }

  public static TalonFX createDefaultTalon(int deviceId, String canBus) {
    return createDefaultTalon(deviceId, canBus, true);
  }

  /**
   * Creates a TalonFX with an option to skip applying the default configuration.
   *
   * @param deviceId The CAN device ID.
   * @param canBus The CAN bus name ("rio", "canivore", etc.).
   * @param applyConfig If true, the default configuration is applied.
   * @return A configured TalonFX instance.
   */
  public static TalonFX createDefaultTalon(int deviceId, String canBus, boolean applyConfig) {
    TalonFX talon = createTalon(deviceId, canBus);
    if (applyConfig) {
      PhoenixUtil.tryUntilOk(
          configTimeoutRetries, () -> talon.getConfigurator().apply(getDefaultConfig()));
    }
    return talon;
  }

  /**
   * Creates a TalonFX configured to permanently follow another TalonFX on the same CAN bus.
   *
   * @param followerId The CAN device ID of the follower TalonFX.
   * @param masterId The CAN device ID of the master TalonFX.
   * @param canBus The CAN bus name for both motors.
   * @param opposeMasterDirection True if the follower should oppose the master's direction.
   * @return A configured follower TalonFX instance.
   */
  public static TalonFX createPermanentFollowerTalon(
      int followerId, int masterId, String canBus, boolean opposeMasterDirection) {
    final TalonFX talon = createTalon(followerId, canBus);
    talon.setControl(new Follower(masterId, opposeMasterDirection));
    return talon;
  }

  /**
   * Gets the default configuration for a TalonFX.
   *
   * @return A TalonFXConfiguration object with default settings.
   */
  public static TalonFXConfiguration getDefaultConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = defaultNeutralMode;
    config.MotorOutput.Inverted = defaultInvertValue;
    config.MotorOutput.DutyCycleNeutralDeadband = defaultNeutralDeadband;
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

  private static TalonFX createTalon(int deviceId, String canBus) {
    TalonFX talon = new TalonFX(deviceId, canBus);
    talon.clearStickyFaults();
    return talon;
  }
}
