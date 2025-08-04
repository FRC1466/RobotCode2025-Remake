// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOInputsAutoLogged;
import frc.robot.subsystems.sensors.CoralSensorIO;
import frc.robot.subsystems.sensors.CoralSensorIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final RollerSystemIO endEffectorIO;
  private final RollerSystemIOInputsAutoLogged endEffectorInputs =
      new RollerSystemIOInputsAutoLogged();
  private final RollerSystemIO starWheelIO;
  private final RollerSystemIOInputsAutoLogged starWheelInputs =
      new RollerSystemIOInputsAutoLogged();
  private final CoralSensorIO coralSensorIO;
  private final CoralSensorIOInputsAutoLogged coralSensorInputs =
      new CoralSensorIOInputsAutoLogged();

  public Intake(RollerSystemIO endEffectorRollerSystemIO, RollerSystemIO starWheelRollerSystemIO, CoralSensorIO coralSensorIO) {
    this.endEffectorIO = endEffectorRollerSystemIO;
    this.starWheelIO = starWheelRollerSystemIO;
    this.coralSensorIO = coralSensorIO;
  }

  public enum WantedState {
    INTAKE_CORAL,
    GRIP_CORAL,
    OUTTAKE_CORAL,
    OUTTAKE_CORAL_L1,
    BACKUP_CORAL,
    INTAKE_ALGAE,
    HOLD_ALGAE,
    HOLD_ALGAE_HARDER,
    EJECT_ALGAE,
    OFF
  }

  private enum SystemState {
    INTAKING_CORAL,
    GRIPPING_CORAL,
    OUTTAKING_CORAL,
    OUTTAKING_CORAL_L1,
    BACKING_UP_CORAL,
    INTAKING_ALGAE,
    HOLDING_ALGAE,
    HOLDING_ALGAE_HARDER,
    EJECTING_ALGAE,
    OFF
  }

  private WantedState wantedState = WantedState.OFF;

  private SystemState systemState = SystemState.OFF;

  private boolean hasAlgaeEntered = false;

  @Override
  public void periodic() {
    endEffectorIO.updateInputs(endEffectorInputs);
    Logger.processInputs("Subsystems/Intake/EndEffectorRoller", endEffectorInputs);
    starWheelIO.updateInputs(starWheelInputs);
    Logger.processInputs("Subsystems/Intake/StarWheelRoller", starWheelInputs);
    coralSensorIO.updateInputs(coralSensorInputs);
    Logger.processInputs("Subsystems/Intake/CoralSensor", coralSensorInputs);

    systemState = handleStateTransition();
    applyState();
    Logger.recordOutput("Subsystems/Intake/SystemState", systemState);
    Logger.recordOutput("Subsystems/Intake/WantedState", wantedState);
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case INTAKE_ALGAE:
        {
          if (hasAlgaeEntered) {
            yield SystemState.HOLDING_ALGAE;
          } else {
            hasAlgaeEntered =
                endEffectorInputs.data.torqueCurrentAmps() >= currentThresholdAlgaeIntake;
            if (hasAlgaeEntered) {
              yield SystemState.HOLDING_ALGAE_HARDER;
            }
          }
          yield SystemState.INTAKING_ALGAE;
        }
      case HOLD_ALGAE:
        {
          hasAlgaeEntered =
              endEffectorInputs.data.torqueCurrentAmps() >= currentThresholdAlgaeIntake;
          yield SystemState.HOLDING_ALGAE;
        }
      case HOLD_ALGAE_HARDER:
        {
          hasAlgaeEntered = true;
          yield SystemState.HOLDING_ALGAE_HARDER;
        }
      case EJECT_ALGAE:
        {
          hasAlgaeEntered = false;
          yield SystemState.EJECTING_ALGAE;
        }
      case INTAKE_CORAL:
        {
          hasAlgaeEntered = false;
          yield SystemState.INTAKING_CORAL;
        }
      case GRIP_CORAL:
        {
          hasAlgaeEntered = false;
          yield SystemState.GRIPPING_CORAL;
        }
      case OUTTAKE_CORAL:
        {
          hasAlgaeEntered = false;
          yield SystemState.OUTTAKING_CORAL;
        }
      case OUTTAKE_CORAL_L1:
        {
          hasAlgaeEntered = false;
          yield SystemState.OUTTAKING_CORAL_L1;
        }
      case BACKUP_CORAL:
        {
          hasAlgaeEntered = false;
          yield SystemState.BACKING_UP_CORAL;
        }
      default:
        {
          hasAlgaeEntered = false;
          yield SystemState.OFF;
        }
    };
  }

  private void applyState() {
    RollerVoltages voltages = new RollerVoltages(0, 0);

    switch (systemState) {
      case INTAKING_CORAL:
        voltages = coralIntake;
        break;
      case GRIPPING_CORAL:
        voltages = coralGrip;
        break;
      case OUTTAKING_CORAL:
        voltages = coralOuttake;
        break;
      case OUTTAKING_CORAL_L1:
        voltages = coralOuttakeL1;
        break;
      case BACKING_UP_CORAL:
        voltages = coralBackup;
        break;
      case INTAKING_ALGAE:
        voltages = algaeIntake;
        break;
      case HOLDING_ALGAE:
        voltages = algaeHold;
        break;
      case HOLDING_ALGAE_HARDER:
        voltages = algaeHold;
        break;
      case EJECTING_ALGAE:
        voltages = algaeEject;
        break;
      default:
        voltages = new RollerVoltages(0.0, 0.0);
        break;
    }

    endEffectorIO.runVolts(voltages.endEffectorVoltage);
    starWheelIO.runVolts(voltages.starWheelVoltage);
  }

  public boolean hasCoral() {
    return coralSensorInputs.data.distanceMeters() < distanceThresholdCoralIntake;
  }

  public boolean hasAlgae() {
    return hasAlgaeEntered;
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }
}
