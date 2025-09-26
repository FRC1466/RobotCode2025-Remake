package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator.ElevatorProfile;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {

  private final TalonFX masterTalonFX = new TalonFX(ElevatorConstants.masterMotorId);
  private final TalonFX followerTalonFX = new TalonFX(ElevatorConstants.followerMotorId);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withUpdateFreqHz(0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withUpdateFreqHz(0);

  // Master signals
  private final StatusSignal<Angle> masterPosition;
  private final StatusSignal<AngularVelocity> masterVelocity;
  private final StatusSignal<AngularAcceleration> masterAcceleration;
  private final StatusSignal<Voltage> masterAppliedVolts;
  private final StatusSignal<Current> masterStatorCurrent;
  private final StatusSignal<Current> masterSupplyCurrent;
  private final StatusSignal<Temperature> masterTemp;

  // Follower signals
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerStatorCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Temperature> followerTemp;

  private final Debouncer masterConnectedDebouncer = new Debouncer(0.4);
  private final Debouncer followerConnectedDebouncer = new Debouncer(0.4);

  public ElevatorIOTalonFX() {
    // Current limits
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.supplyCurrentLimit.get();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.statorCurrentLimit.get();

    // PID + feedforward
    config.Slot0 = new Slot0Configs()
        .withKP(ElevatorConstants.kP.get())
        .withKI(ElevatorConstants.kI.get())
        .withKD(ElevatorConstants.kD.get())
        .withKS(ElevatorConstants.kS.get())
        .withKG(ElevatorConstants.kG.get());

    // Neutral mode
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Initial Motion Magic profile
    applyMotionProfile(ElevatorProfile.DEFAULT, config);

    tryUntilOk(5, () -> masterTalonFX.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> followerTalonFX.getConfigurator().apply(config, 0.25));

    followerTalonFX.setControl(new Follower(masterTalonFX.getDeviceID(), true));

    // Signals
    masterPosition = masterTalonFX.getPosition();
    masterVelocity = masterTalonFX.getVelocity();
    masterAcceleration = masterTalonFX.getAcceleration();
    masterAppliedVolts = masterTalonFX.getMotorVoltage();
    masterStatorCurrent = masterTalonFX.getStatorCurrent();
    masterSupplyCurrent = masterTalonFX.getSupplyCurrent();
    masterTemp = masterTalonFX.getDeviceTemp();

    followerAppliedVolts = followerTalonFX.getMotorVoltage();
    followerStatorCurrent = followerTalonFX.getStatorCurrent();
    followerSupplyCurrent = followerTalonFX.getSupplyCurrent();
    followerTemp = followerTalonFX.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        masterPosition,
        masterVelocity,
        masterAcceleration,
        masterAppliedVolts,
        masterStatorCurrent,
        masterSupplyCurrent,
        followerAppliedVolts,
        followerStatorCurrent,
        followerSupplyCurrent);
    masterTemp.setUpdateFrequency(10);
    followerTemp.setUpdateFrequency(10);

    ParentDevice.optimizeBusUtilizationForAll(masterTalonFX, followerTalonFX);

    PhoenixUtil.registerSignals(
        false,
        masterPosition,
        masterVelocity,
        masterAcceleration,
        masterAppliedVolts,
        masterStatorCurrent,
        masterSupplyCurrent,
        masterTemp,
        followerAppliedVolts,
        followerStatorCurrent,
        followerSupplyCurrent,
        followerTemp);
  }

  private void applyMotionProfile(ElevatorProfile profile, TalonFXConfiguration config) {
    config.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.elevatorMetersToRotations(profile.velocity.get());
    config.MotionMagic.MotionMagicAcceleration =
        ElevatorConstants.elevatorMetersToRotations(profile.acceleration.get());
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        masterPosition,
        masterVelocity,
        masterAcceleration,
        masterAppliedVolts,
        masterStatorCurrent,
        masterSupplyCurrent,
        masterTemp,
        followerAppliedVolts,
        followerStatorCurrent,
        followerSupplyCurrent,
        followerTemp);

    boolean masterGood =
        BaseStatusSignal.isAllGood(
            masterPosition, masterVelocity, masterAppliedVolts, masterSupplyCurrent, masterTemp);
    boolean followerGood =
        BaseStatusSignal.isAllGood(
            followerAppliedVolts, followerStatorCurrent, followerSupplyCurrent, followerTemp);

    double rot = masterPosition.getValueAsDouble();
    double rps = masterVelocity.getValueAsDouble();
    double rps2 = masterAcceleration.getValueAsDouble();

    inputs.elevatorPositionMeters = ElevatorConstants.elevatorRotationsToMeters(rot);
    inputs.elevatorVelocityMetersPerSec = ElevatorConstants.elevatorRotationsToMeters(rps);
    inputs.elevatorAccelerationMetersPerSecSquared =
        ElevatorConstants.elevatorRotationsToMeters(rps2);

    inputs.elevatorMasterAppliedVolts = masterAppliedVolts.getValueAsDouble();
    inputs.elevatorMasterStatorCurrentAmps = masterStatorCurrent.getValueAsDouble();
    inputs.elevatorMasterSupplyCurrentAmps = masterSupplyCurrent.getValueAsDouble();
    inputs.elevatorMasterMotorTemp = masterTemp.getValueAsDouble();

    inputs.elevatorFollowerAppliedVolts = followerAppliedVolts.getValueAsDouble();
    inputs.elevatorFollowerStatorCurrentAmps = followerStatorCurrent.getValueAsDouble();
    inputs.elevatorFollowerSupplyCurrentAmps = followerSupplyCurrent.getValueAsDouble();
    inputs.elevatorFollowerMotorTemp = followerTemp.getValueAsDouble();

    inputs.elevatorMasterConnected = masterConnectedDebouncer.calculate(masterGood);
    inputs.elevatorFollowerConnected = followerConnectedDebouncer.calculate(followerGood);
  }

  @Override
  public void setTargetPosition(double positionInMeters) {
    masterTalonFX.setControl(
        positionVoltage.withPosition(
            ElevatorConstants.elevatorMetersToRotations(positionInMeters)));
  }

  @Override
  public void resetElevatorPosition(double positionInMeters) {
    double rotations = ElevatorConstants.elevatorMetersToRotations(positionInMeters);
    tryUntilOk(5, () -> masterTalonFX.setPosition(rotations));
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
    MotionMagicConfigs mm = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(
            ElevatorConstants.elevatorMetersToRotations(elevatorProfile.velocity.get()))
        .withMotionMagicAcceleration(
            ElevatorConstants.elevatorMetersToRotations(elevatorProfile.acceleration.get()));
    tryUntilOk(5, () -> masterTalonFX.getConfigurator().apply(mm));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    Slot0Configs slot =
        new Slot0Configs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKS(config.Slot0.kS)
            .withKG(config.Slot0.kG);
    tryUntilOk(5, () -> masterTalonFX.getConfigurator().apply(slot));
  }
}