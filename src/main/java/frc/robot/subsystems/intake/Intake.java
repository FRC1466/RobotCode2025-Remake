// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.frc2910.robot.subsystems.intake;

import static org.frc2910.robot.constants.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.RollerSystemIO;
import org.frc2910.robot.util.SubsystemDataProcessor;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final RollerSystemIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
    SubsystemDataProcessor.createAndStartSubsystemDataProcessor(
        () -> {
          synchronized (inputs) {
            io.updateInputs(inputs);
          }
        },
        io);
  }

  public enum WantedState {
    COLLECT_ALGAE,
    HOLD_ALGAE,
    HOLD_ALGAE_HARDER,
    EJECT_ALGAE,
    EJECT_ALGAE_PROCESSOR,
    COLLECT_CORAL_STRAIGHT,
    COLLECT_CORAL_FROM_STATION,
    COLLECT_CORAL_HORIZONTAL,
    HOLD_CORAL_STRAIGHT,
    HOLD_CORAL_HORIZONTAL,
    EJECT_CORAL_L1,
    EJECT_CORAL_FORWARDS,
    EJECT_CORAL_BACKWARDS,
    INDEX_CORAL_FORWARD,
    INDEX_CORAL_BACKWARD,
    OFF
  }

  private enum SystemState {
    COLLECTING_ALGAE,
    HOLDING_ALGAE,
    HOLDING_ALGAE_HARDER,
    EJECTING_ALGAE,
    EJECTING_ALGAE_PROCESSOR,
    COLLECTING_CORAL_STRAIGHT,
    COLLECTING_CORAL_FROM_STATION,
    COLLECTING_CORAL_HORIZONTAL,
    HOLDING_CORAL_STRAIGHT,
    HOLDING_CORAL_HORIZONTAL,
    EJECTING_CORAL_L1_STRAIGHT,
    EJECTING_CORAL_L1_HORIZONTAL,
    EJECTING_CORAL_FORWARDS,
    EJECTING_CORAL_BACKWARDS,
    INDEXING_CORAL_FORWARD,
    INDEXING_CORAL_BACKWARD,
    INDEXING_CORAL_FROM_STRAIGHT_TO_HORIZONTAL,
    INDEX_CORAL_LEFT,
    INDEX_CORAL_RIGHT,
    INDEX_CORAL_RIGHT_FASTER,
    OFF
  }

  private WantedState wantedState = WantedState.OFF;

  private SystemState systemState = SystemState.OFF;

  private boolean isAllowedToCheckIfAlgaeHasEntered = false;
  private boolean hasAlgaeEntered = false;

  @Override
  public void periodic() {
    synchronized (inputs) {
      Logger.processInputs("Subsystems/Intake", inputs);

      systemState = handleStateTransition();
      applyState();
      Logger.recordOutput("Subsystems/Intake/SystemState", systemState);
      Logger.recordOutput("Subsystems/Intake/WantedState", wantedState);
    }
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case COLLECT_ALGAE:
        {
          if (hasAlgaeEntered) {
            yield SystemState.HOLDING_ALGAE;
          } else if (!isAllowedToCheckIfAlgaeHasEntered) {
            isAllowedToCheckIfAlgaeHasEntered =
                inputs.topSupplyCurrent <= TOP_ROLLER_CURRENT_THRESHOLD_FOR_ALGAE_DETECTION
                    && inputs.topVelocityRPS
                        <= TOP_ROLLER_VELOCITY_RPS_THRESHOLD_FOR_ALGAE_DETECTION_ALLOWANCE;
          } else {
            hasAlgaeEntered =
                inputs.topVelocityRPS
                    >= TOP_ROLLER_VELOCITY_RPS_THRESHOLD_FOR_ALGAE_DETECTION_WHILE_INTAKING;
            if (hasAlgaeEntered) {
              yield SystemState.HOLDING_ALGAE_HARDER;
            }
          }
          yield SystemState.COLLECTING_ALGAE;
        }
      case HOLD_ALGAE:
        {
          hasAlgaeEntered =
              inputs.topVelocityRPS
                  >= TOP_ROLLER_VELOCITY_RPS_THRESHOLD_FOR_ALGAE_DETECTION_WHILE_HOLDING;
          isAllowedToCheckIfAlgaeHasEntered = false;
          yield SystemState.HOLDING_ALGAE;
        }
      case HOLD_ALGAE_HARDER:
        {
          hasAlgaeEntered = true;
          isAllowedToCheckIfAlgaeHasEntered = false;
          yield SystemState.HOLDING_ALGAE_HARDER;
        }
      case EJECT_ALGAE_PROCESSOR:
        {
          isAllowedToCheckIfAlgaeHasEntered = false;
          hasAlgaeEntered = false;
          yield SystemState.EJECTING_ALGAE_PROCESSOR;
        }
      case EJECT_ALGAE:
        {
          isAllowedToCheckIfAlgaeHasEntered = false;
          hasAlgaeEntered = false;
          yield SystemState.EJECTING_ALGAE;
        }
      case COLLECT_CORAL_STRAIGHT:
        {
          synchronized (inputs) {
            if (inputs.isBackCANTRangeTripped) {
              yield SystemState.HOLDING_CORAL_STRAIGHT;
            } else if (inputs.isFrontLeftCANRangeTripped && inputs.isFrontRightCANRangeTripped) {
              yield SystemState.INDEX_CORAL_RIGHT_FASTER;
            } else {
              yield SystemState.COLLECTING_CORAL_STRAIGHT;
            }
          }
        }
      case COLLECT_CORAL_FROM_STATION:
        {
          synchronized (inputs) {
            if (inputs.isBackCANTRangeTripped) {
              yield SystemState.HOLDING_CORAL_STRAIGHT;
            } else if (inputs.isFrontLeftCANRangeTripped && inputs.isFrontRightCANRangeTripped) {
              yield SystemState.INDEX_CORAL_RIGHT_FASTER;
            } else {
              yield SystemState.COLLECTING_CORAL_FROM_STATION;
            }
          }
        }
      case COLLECT_CORAL_HORIZONTAL:
        {
          synchronized (inputs) {
            if (!inputs.isBackCANTRangeTripped
                && !inputs.isFrontCANRangeTripped
                && !inputs.isFrontRightCANRangeTripped
                && !inputs.isFrontLeftCANRangeTripped) {
              yield SystemState.COLLECTING_CORAL_HORIZONTAL;
            } else if (hasHorizontalCoral()) {
              yield SystemState.HOLDING_CORAL_HORIZONTAL;
            } else if (inputs.isBackCANTRangeTripped) {
              yield SystemState.INDEXING_CORAL_FROM_STRAIGHT_TO_HORIZONTAL;
            } else if (inputs.isFrontLeftCANRangeTripped) {
              yield SystemState.INDEX_CORAL_RIGHT;
            } else if (inputs.isFrontRightCANRangeTripped || inputs.isFrontCANRangeTripped) {
              yield SystemState.INDEX_CORAL_LEFT;
            }
          }
        }
      case HOLD_CORAL_STRAIGHT:
        {
          synchronized (inputs) {
            if (inputs.isFrontCANRangeTripped && inputs.isBackCANTRangeTripped) {
              yield SystemState.HOLDING_CORAL_STRAIGHT;
            } else if (inputs.isBackCANTRangeTripped) {
              yield SystemState.INDEXING_CORAL_FORWARD;
            } else if (inputs.isFrontCANRangeTripped) {
              yield SystemState.INDEXING_CORAL_BACKWARD;
            }
          }
        }
      case HOLD_CORAL_HORIZONTAL:
        {
          synchronized (inputs) {
            yield SystemState.HOLDING_CORAL_HORIZONTAL;
          }
        }
      case EJECT_CORAL_L1:
        {
          if (inputs.isFrontCANRangeTripped || inputs.isBackCANTRangeTripped) {
            yield SystemState.EJECTING_CORAL_L1_STRAIGHT;
          } else {
            yield SystemState.EJECTING_CORAL_L1_HORIZONTAL;
          }
        }
      case EJECT_CORAL_FORWARDS:
        {
          yield SystemState.EJECTING_CORAL_FORWARDS;
        }
      case EJECT_CORAL_BACKWARDS:
        {
          yield SystemState.EJECTING_CORAL_BACKWARDS;
        }
      case INDEX_CORAL_FORWARD:
        {
          if (inputs.isBackCANTRangeTripped) {
            yield SystemState.INDEXING_CORAL_FORWARD;
          } else {
            yield SystemState.HOLDING_CORAL_STRAIGHT;
          }
        }
      case INDEX_CORAL_BACKWARD:
        {
          if (inputs.isFrontCANRangeTripped) {
            yield SystemState.INDEXING_CORAL_BACKWARD;
          } else {
            yield SystemState.HOLDING_CORAL_STRAIGHT;
          }
        }
      default:
        {
          isAllowedToCheckIfAlgaeHasEntered = false;
          hasAlgaeEntered = false;
          yield SystemState.OFF;
        }
    };
  }

  private void applyState() {
    double verticalRightMotorVoltage = 0.0;
    double verticalLeftMotorVoltage = 0.0;
    double topMotorVoltage = 0.0;

    switch (systemState) {
      case EJECTING_ALGAE:
        topMotorVoltage = EJECTING_TOP_ALGAE_VOLTAGE;
        break;
      case EJECTING_ALGAE_PROCESSOR:
        topMotorVoltage = EJECTING_PROCESSOR_TOP_ALGAE_VOLTAGE;
        break;
      case COLLECTING_ALGAE:
        topMotorVoltage = COLLECTING_TOP_ALGAE_VOLTAGE;
        break;
      case HOLDING_ALGAE:
        topMotorVoltage = HOLDING_TOP_ALGAE_VOLTAGE;
        break;
      case HOLDING_ALGAE_HARDER:
        topMotorVoltage = HOLDING_TOP_ALGAE_HARDER_VOLTAGE;
        break;
      case EJECTING_CORAL_L1_STRAIGHT:
        topMotorVoltage = EJECTING_TOP_CORAL_VOLTAGE_L1;
        verticalRightMotorVoltage = EJECTING_VERTICAL_CORAL_VOLTAGE_L1;
        verticalLeftMotorVoltage = EJECTING_VERTICAL_CORAL_VOLTAGE_L1;
        break;
      case EJECTING_CORAL_L1_HORIZONTAL:
        topMotorVoltage = EJECTING_TOP_CORAL_VOLTAGE_HORIZONTAL_L1;
        break;
      case EJECTING_CORAL_FORWARDS:
        verticalRightMotorVoltage = EJECTING_VERTICAL_CORAL_VOLTAGE;
        verticalLeftMotorVoltage = EJECTING_VERTICAL_CORAL_VOLTAGE;
        topMotorVoltage = EJECTING_TOP_CORAL_VOLTAGE;
        break;
      case EJECTING_CORAL_BACKWARDS:
        verticalRightMotorVoltage = -EJECTING_VERTICAL_CORAL_VOLTAGE;
        verticalLeftMotorVoltage = -EJECTING_VERTICAL_CORAL_VOLTAGE;
        topMotorVoltage = -EJECTING_TOP_CORAL_VOLTAGE;
        break;
      case COLLECTING_CORAL_STRAIGHT:
        verticalRightMotorVoltage = COLLECTING_RIGHT_VERTICAL_CORAL_VOLTAGE;
        verticalLeftMotorVoltage = COLLECTING_LEFT_VERTICAL_CORAL_VOLTAGE;
        topMotorVoltage = COLLECTING_TOP_CORAL_VOLTAGE;
        break;
      case COLLECTING_CORAL_FROM_STATION:
        verticalRightMotorVoltage = COLLECTING_FROM_STATION_VERTICAL_CORAL_VOLTAGE;
        verticalLeftMotorVoltage = COLLECTING_FROM_STATION_VERTICAL_CORAL_VOLTAGE;
        topMotorVoltage = COLLECTING_TOP_CORAL_VOLTAGE;
        break;

      case COLLECTING_CORAL_HORIZONTAL:
        topMotorVoltage = COLLECTING_TOP_CORAL_VOLTAGE;
        break;
      case HOLDING_CORAL_STRAIGHT:
        break;
      case HOLDING_CORAL_HORIZONTAL:
        topMotorVoltage = 5.0;
        break;
      case INDEXING_CORAL_FORWARD:
        verticalRightMotorVoltage = INDEXING_CORAL_FORWARD_VERTICAL_VOLTAGE;
        verticalLeftMotorVoltage = INDEXING_CORAL_FORWARD_VERTICAL_VOLTAGE;
        topMotorVoltage = INDEXING_CORAL_FORWARD_TOP_VOLTAGE;
        break;
      case INDEXING_CORAL_BACKWARD:
        verticalRightMotorVoltage = INDEXING_CORAL_BACKWARD_VERTICAL_VOLTAGE;
        verticalLeftMotorVoltage = INDEXING_CORAL_BACKWARD_VERTICAL_VOLTAGE;
        topMotorVoltage = INDEXING_CORAL_BACKWARD_TOP_VOLTAGE;
        break;
      case INDEX_CORAL_RIGHT:
        verticalRightMotorVoltage = -5.0;
        verticalLeftMotorVoltage = 5.0;
        topMotorVoltage = COLLECTING_TOP_CORAL_VOLTAGE;
        break;
      case INDEX_CORAL_RIGHT_FASTER:
        verticalRightMotorVoltage = -10.0;
        verticalLeftMotorVoltage = 10.0;
        topMotorVoltage = COLLECTING_TOP_CORAL_VOLTAGE;
        break;
      case INDEX_CORAL_LEFT:
        verticalRightMotorVoltage = 5.0;
        verticalLeftMotorVoltage = -5.0;
        topMotorVoltage = COLLECTING_TOP_CORAL_VOLTAGE;
        break;
      case INDEXING_CORAL_FROM_STRAIGHT_TO_HORIZONTAL:
        verticalRightMotorVoltage = COLLECTING_RIGHT_VERTICAL_CORAL_VOLTAGE;
        verticalLeftMotorVoltage = EJECTING_VERTICAL_CORAL_VOLTAGE;
        topMotorVoltage = COLLECTING_TOP_CORAL_VOLTAGE;
        break;
      case OFF:
      default:
        verticalRightMotorVoltage = 0.0;
        topMotorVoltage = 0.0;
        break;
    }

    io.setTopMotorVoltage(topMotorVoltage);
    io.setLeftVerticalMotorVoltage(verticalLeftMotorVoltage);
    io.setRightVerticalMotorVoltage(verticalRightMotorVoltage);
  }

  public boolean hasStraightCoral() {
    synchronized (inputs) {
      return (inputs.isFrontCANRangeTripped || inputs.isBackCANTRangeTripped);
    }
  }

  public boolean hasHorizontalCoral() {
    synchronized (inputs) {
      return inputs.isFrontCANRangeTripped
          && inputs.isFrontLeftCANRangeTripped
          && inputs.isFrontRightCANRangeTripped;
    }
  }

  public boolean isBackCanRangeTripped() {
    synchronized (inputs) {
      return inputs.isBackCANTRangeTripped;
    }
  }

  public boolean hasAlgae() {
    return hasAlgaeEntered;
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }
}
