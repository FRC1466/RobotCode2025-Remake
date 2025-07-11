// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.manipulator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.commands.AutoScoreCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOInputsAutoLogged;
import frc.robot.subsystems.sensors.CoralSensorIO;
import frc.robot.subsystems.sensors.CoralSensorIOInputsAutoLogged;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Manipulator {
  public static final Rotation2d minAngle = Rotation2d.fromRadians(0);
  public static final Rotation2d maxAngle = Rotation2d.fromRadians(Math.PI + 2);

  // Tunable numbers
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Manipulator/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Manipulator/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Manipulator/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Manipulator/kG");

  private static final LoggedTunableNumber maxVelocityDegPerSec =
      new LoggedTunableNumber("Manipulator/MaxVelocityDegPerSec", 1500.0);
  private static final LoggedTunableNumber maxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Manipulator/MaxAccelerationDegPerSec2", 2500.0);
  private static final LoggedTunableNumber slowMaxVelocityDegPerSec =
      new LoggedTunableNumber("Manipulator/SlowMaxVelocityDegPerSec", 800.0);
  private static final LoggedTunableNumber slowMaxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Manipulator/SlowMaxAccelerationDegPerSec2", 1500.0);

  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      new LoggedTunableNumber("Manipulator/StaticCharacterizationVelocityThresh", 0.1);
  private static final LoggedTunableNumber staticCharacterizationRampRate =
      new LoggedTunableNumber("Manipulator/StaticCharacterizationRampRate", 0.2);

  private static final LoggedTunableNumber algaeCurrentThresh =
      new LoggedTunableNumber("Dispenser/AlgaeCurrentThreshold", 10.0);
  public static final LoggedTunableNumber throwInterpolationValue =
      new LoggedTunableNumber("Dispenser/AlgaeCurrentThreshold", 0.6);
  private static final LoggedTunableNumber coralProxThreshold =
      new LoggedTunableNumber("Dispenser/CoralProxThresh", 0.06);

  public static final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Manipulator/Tolerance", Math.PI / 4);
  public static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Manipulator/HomingTimeSecs", 0.2);
  public static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Manipulator/HomingVolts", 3.0);
  public static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber("Manipulator/HomingVelocityThreshold", 0.1);
  public static final LoggedTunableNumber simIntakingTime =
      new LoggedTunableNumber("Manipulator/SimIntakingTime", 1);
  public static final LoggedTunableNumber simIntakingTimeAlgae =
      new LoggedTunableNumber("Manipulator/SimIntakingTimeAlgae", 1);

  public static final LoggedTunableNumber[] IDLE = {
    new LoggedTunableNumber("Manipulator/Mailbox/IDLE", 0),
    new LoggedTunableNumber("Manipulator/Funnel/IDLE", 0),
  };
  public static final LoggedTunableNumber[] ALGAEGRAB = {
    new LoggedTunableNumber("Manipulator/Mailbox/ALGAEGRAB", 3.0),
    new LoggedTunableNumber("Manipulator/Funnel/ALGAEGRAB", 0),
  };
  public static final LoggedTunableNumber[] ALGAEHOLD = {
    new LoggedTunableNumber("Manipulator/Mailbox/ALGAEHOLD", .4),
    new LoggedTunableNumber("Manipulator/Funnel/ALGAEHOLD", 0),
  };
  public static final LoggedTunableNumber[] ALGAENET = {
    new LoggedTunableNumber("Manipulator/Mailbox/ALGAENET", -2.5),
    new LoggedTunableNumber("Manipulator/Funnel/ALGAENET", 0),
  };
  public static final LoggedTunableNumber[] CORALINTAKE = {
    new LoggedTunableNumber("Manipulator/Mailbox/CORALINTAKE", -1.25),
    new LoggedTunableNumber("Manipulator/Funnel/CORALINTAKE", -4),
  };
  public static final LoggedTunableNumber[] CORALL4GRIP = {
    new LoggedTunableNumber("Manipulator/Mailbox/CORALL4GRIP", .2),
    new LoggedTunableNumber("Manipulator/Funnel/CORALL4GRIP", 0),
  };
  public static final LoggedTunableNumber[] CORALOUTTAKE = {
    new LoggedTunableNumber("Manipulator/Mailbox/CORALOUTTAKE", -10),
    new LoggedTunableNumber("Manipulator/Funnel/CORALOUTTAKE", 0),
  };
  public static final LoggedTunableNumber[] CORALEJECT = {
    new LoggedTunableNumber("Manipulator/Mailbox/CORALEJECT", -12),
    new LoggedTunableNumber("Manipulator/Funnel/CORALEJECT", -12),
  };
  public static final LoggedTunableNumber[] CORALBACKUP = {
    new LoggedTunableNumber("Manipulator/Mailbox/CORALBACKUP", .5),
    new LoggedTunableNumber("Manipulator/Funnel/CORALBACKUP", 0),
  };
  public static final LoggedTunableNumber[] CORALL1 = {
    new LoggedTunableNumber("Manipulator/Mailbox/CORALL1", 4),
    new LoggedTunableNumber("Manipulator/Funnel/CORALBACKUP", 0),
  };

  static {
    switch (Constants.getRobot()) {
      case SIMBOT -> {
        kP.initDefault(4000);
        kD.initDefault(1000);
        kS.initDefault(1.2);
        kG.initDefault(0.0);
      }
      default -> {
        kP.initDefault(50);
        kD.initDefault(3);
        kS.initDefault(0);
        kG.initDefault(0);
      }
    }
  }

  public enum MailboxGoal {
    IDLE,
    ALGAEGRAB,
    ALGAEHOLD,
    ALGAENET,
    CORALINTAKE,
    CORALL4GRIP,
    CORALOUTTAKE,
    CORALBACKUP,
    CORALEJECT,
    CORALL1
  }

  // Hardware
  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  private final RollerSystemIO mailboxIO;
  private final RollerSystemIOInputsAutoLogged mailboxInputs = new RollerSystemIOInputsAutoLogged();
  private final RollerSystemIO funnelRollerIO;
  private final RollerSystemIOInputsAutoLogged funnelRollerInputs =
      new RollerSystemIOInputsAutoLogged();
  private final CoralSensorIO coralSensorIO;
  private final CoralSensorIOInputsAutoLogged coralSensorInputs =
      new CoralSensorIOInputsAutoLogged();
  private final Drive drive;

  // Overrides
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disabledOverride = () -> false;

  @AutoLogOutput(key = "Manipulator/PivotBrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  private TrapezoidProfile profile;
  private TrapezoidProfile slowProfile;
  @Getter private State setpoint = new State();
  private DoubleSupplier goal = () -> 0.0;
  private boolean stopProfile = false;
  @Getter private boolean shouldEStop = false;
  @Setter private boolean isEStopped = false;
  @Setter private boolean isIntaking = false;
  @Setter private boolean isIntakingAlgae = false;

  @Getter
  @AutoLogOutput(key = "Manipulator/Profile/AtGoal")
  private boolean atGoal = false;

  @AutoLogOutput @Setter private MailboxGoal mailboxGoal = MailboxGoal.IDLE;

  @AutoLogOutput
  @Accessors(fluent = true)
  @Getter()
  private boolean hasCoral = false;

  @AutoLogOutput
  @Accessors(fluent = true)
  @Getter()
  private boolean hasAlgae = false;

  private static final double coralDebounceTime = 0.04;
  private static final double algaeDebounceTime = 0.6;
  private Debouncer coralDebouncer = new Debouncer(coralDebounceTime, DebounceType.kFalling);
  private Debouncer algaeDebouncer = new Debouncer(algaeDebounceTime, DebounceType.kBoth);
  private Debouncer toleranceDebouncer = new Debouncer(0.25, DebounceType.kRising);
  private final SlewRateLimiter algaeCurrentFilter = new SlewRateLimiter(50);

  // Disconnected alerts
  private final Alert pivotMotorDisconnectedAlert =
      new Alert("Manipulator pivot motor disconnected!", Alert.AlertType.kWarning);
  private final Alert pivotEncoderDisconnectedAlert =
      new Alert("Manipulator pivot encoder disconnected!", Alert.AlertType.kWarning);
  private final Alert mailboxDisconnectedAlert =
      new Alert("Manipulator mailbox disconnected!", Alert.AlertType.kWarning);
  private final Alert funnelRollerDisconnectedAlert =
      new Alert("Manipulator funnel disconnected!", Alert.AlertType.kWarning);

  private final Timer simIntakingTimer = new Timer();
  private final Timer simIntakingTimerAlgae = new Timer();
  private boolean lastAlgaeButtonPressed = false;
  private boolean lastCoralButtonPressed = false;

  public Manipulator(
      PivotIO pivotIO,
      RollerSystemIO mailboxIO,
      RollerSystemIO funnelRollerIO,
      CoralSensorIO coralSensorIO,
      Drive drive) {
    this.pivotIO = pivotIO;
    this.mailboxIO = mailboxIO;
    this.funnelRollerIO = funnelRollerIO;
    this.coralSensorIO = coralSensorIO;
    this.drive = drive;

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(maxVelocityDegPerSec.get()),
                Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
  }

  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Manipulator/Pivot", pivotInputs);
    mailboxIO.updateInputs(mailboxInputs);
    Logger.processInputs("Manipulator/Mailbox", mailboxInputs);
    funnelRollerIO.updateInputs(funnelRollerInputs);
    Logger.processInputs("Manipulator/Funnel Roller", funnelRollerInputs);
    coralSensorIO.updateInputs(coralSensorInputs);
    Logger.processInputs("Manipulator/CoralSensor", coralSensorInputs);

    pivotMotorDisconnectedAlert.set(
        !pivotInputs.data.motorConnected()
            && Constants.getRobot() == RobotType.COMPBOT
            && !Robot.isJITing());
    pivotEncoderDisconnectedAlert.set(
        !pivotInputs.data.encoderConnected()
            && Constants.getRobot() == RobotType.COMPBOT
            && !Robot.isJITing());
    mailboxDisconnectedAlert.set(!mailboxInputs.data.connected() && !Robot.isJITing());
    funnelRollerDisconnectedAlert.set(
        !funnelRollerInputs.data.connected()
            && Constants.getRobot() == RobotType.COMPBOT
            && !Robot.isJITing());

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      pivotIO.setPID(kP.get(), 0.0, kD.get());
    }
    if (maxVelocityDegPerSec.hasChanged(hashCode())
        || maxAccelerationDegPerSec2.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(maxVelocityDegPerSec.get()),
                  Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
    }
    if (slowMaxVelocityDegPerSec.hasChanged(hashCode())
        || slowMaxAccelerationDegPerSec2.hasChanged(hashCode())) {
      slowProfile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(slowMaxVelocityDegPerSec.get()),
                  Units.degreesToRadians(slowMaxAccelerationDegPerSec2.get())));
    }

    // Set coast mode
    setBrakeMode(!coastOverride.getAsBoolean());

    // Run profile
    final boolean shouldRunProfile =
        !stopProfile
            && !coastOverride.getAsBoolean()
            && !disabledOverride.getAsBoolean()
            && !isEStopped
            && DriverStation.isEnabled();
    Logger.recordOutput("Manipulator/RunningProfile", shouldRunProfile);

    // Check if out of tolerance
    boolean outOfTolerance =
        Math.abs(pivotInputs.data.internalPosition().getRadians() - setpoint.position)
            > tolerance.get();
    shouldEStop = toleranceDebouncer.calculate(outOfTolerance && shouldRunProfile);
    if (shouldRunProfile) {
      // Clamp goal
      var goalState =
          new State(
              MathUtil.clamp(goal.getAsDouble(), minAngle.getRadians(), maxAngle.getRadians()),
              0.0);
      setpoint =
          (hasAlgae ? slowProfile : profile)
              .calculate(Constants.loopPeriodSecs, setpoint, goalState);
      pivotIO.runPosition(
          Rotation2d.fromRadians(setpoint.position),
          kS.get() * Math.signum(setpoint.velocity) // Magnitude irrelevant
              + kG.get());

      // Check at goal
      atGoal =
          EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
              && EqualsUtil.epsilonEquals(setpoint.velocity, 0.0);

      // Log state
      Logger.recordOutput("Manipulator/Profile/SetpointAngleRad", setpoint.position);
      Logger.recordOutput("Manipulator/Profile/SetpointAngleRadPerSec", setpoint.velocity);
      Logger.recordOutput("Manipulator/Profile/GoalAngleRad", goalState.position);
    } else {
      // Reset setpoint
      setpoint = new State(pivotInputs.data.encoderAbsolutePosition().getRadians(), 0.0);

      // Clear logs
      Logger.recordOutput("Manipulator/Profile/SetpointAngleRad", 0.0);
      Logger.recordOutput("Manipulator/Profile/SetpointAngleRadPerSec", 0.0);
      Logger.recordOutput("Manipulator/Profile/GoalAngleRad", 0.0);
    }
    if (isEStopped) {
      pivotIO.stop();
    }

    // Run tunnel and gripper
    if (!isEStopped) {
      switch (mailboxGoal) {
        case IDLE -> {
          mailboxIO.stop();
          funnelRollerIO.stop();
        }
        case ALGAEGRAB -> {
          mailboxIO.runVolts(ALGAEGRAB[0].get());
          funnelRollerIO.runVolts(ALGAEGRAB[1].get());
        }
        case ALGAEHOLD -> {
          mailboxIO.runVolts(ALGAEHOLD[0].get());
          funnelRollerIO.runVolts(ALGAEHOLD[1].get());
        }
        case ALGAENET -> {
          mailboxIO.runVolts(ALGAENET[0].get());
          funnelRollerIO.runVolts(ALGAENET[1].get());
        }
        case CORALINTAKE -> {
          mailboxIO.runVolts(CORALINTAKE[0].get());
          funnelRollerIO.runVolts(CORALINTAKE[1].get());
        }
        case CORALL4GRIP -> {
          mailboxIO.runVolts(CORALL4GRIP[0].get());
          funnelRollerIO.runVolts(CORALL4GRIP[1].get());
        }
        case CORALOUTTAKE -> {
          mailboxIO.runVolts(CORALOUTTAKE[0].get());
          funnelRollerIO.runVolts(CORALOUTTAKE[1].get());
        }
        case CORALBACKUP -> {
          mailboxIO.runVolts(CORALBACKUP[0].get());
          funnelRollerIO.runVolts(CORALBACKUP[1].get());
        }
        case CORALEJECT -> {
          mailboxIO.runVolts(CORALEJECT[0].get());
          funnelRollerIO.runVolts(CORALEJECT[1].get());
        }
        case CORALL1 -> {
          mailboxIO.runVolts(CORALL1[0].get());
          funnelRollerIO.runVolts(CORALL1[1].get());
        }
      }
    } else {
      mailboxIO.stop();
      funnelRollerIO.stop();
    }

    // Check algae & coral states
    if (Constants.getRobot() != Constants.RobotType.SIMBOT) {
      if (mailboxGoal == MailboxGoal.ALGAEGRAB || DriverStation.isDisabled()) {
        double limitedTorqueCurrent =
            algaeCurrentFilter.calculate(mailboxInputs.data.torqueCurrentAmps());
        hasAlgae = algaeDebouncer.calculate(limitedTorqueCurrent >= algaeCurrentThresh.get());
        Logger.recordOutput("Manipulator/TorqueCurrentFiltered", limitedTorqueCurrent);
      } else {
        algaeDebouncer.calculate(hasAlgae);
      }
      hasCoral =
          coralDebouncer.calculate(
              coralSensorInputs.data.valid()
                  && coralSensorInputs.data.distanceMeters() < coralProxThreshold.get());
    } else {
      boolean algaeButtonPressed = DriverStation.getStickButtonPressed(2, 1);
      boolean coralButtonPressed = DriverStation.getStickButtonPressed(2, 2);
      if (algaeButtonPressed && !lastAlgaeButtonPressed) {
        hasAlgae = !hasAlgae;
      }
      if (coralButtonPressed && !lastCoralButtonPressed) {
        hasCoral = !hasCoral;
      }
      lastAlgaeButtonPressed = algaeButtonPressed;
      lastCoralButtonPressed = coralButtonPressed;
    }

    var flippedRobot = AllianceFlipUtil.apply(drive.getPose());
    var algaeIntakingError =
        flippedRobot.relativeTo(flippedRobot.nearest(List.of(FieldConstants.Reef.centerFaces)));
    var algaeIceCreamIntakingError =
        flippedRobot.relativeTo(
            flippedRobot.nearest(
                List.of(
                    AutoScoreCommands.getIceCreamIntakePose(new FieldConstants.IceCreamObjective(1))
                        .transformBy(new Transform2d(new Translation2d(0.6, 0), new Rotation2d())),
                    AutoScoreCommands.getIceCreamIntakePose(new FieldConstants.IceCreamObjective(2))
                        .transformBy(new Transform2d(new Translation2d(0.6, 0), new Rotation2d())),
                    AutoScoreCommands.getIceCreamIntakePose(new FieldConstants.IceCreamObjective(3))
                        .transformBy(
                            new Transform2d(new Translation2d(0.6, 0), new Rotation2d())))));
    var intakingError =
        flippedRobot.relativeTo(
            flippedRobot.nearest(
                List.of(
                    FieldConstants.CoralStation.leftCenterFace,
                    FieldConstants.CoralStation.rightCenterFace)));
    if (Constants.getRobot() == Constants.RobotType.SIMBOT) {
      if (isIntaking
          && intakingError.getX() <= Units.inchesToMeters(48.0)
          && Math.abs(intakingError.getY()) <= Units.inchesToMeters(48.0)) {
        hasCoral = simIntakingTimer.hasElapsed(simIntakingTime.get());
      } else {
        simIntakingTimer.restart();
      }

      if (isIntakingAlgae
          && ((Math.abs(algaeIntakingError.getX()) <= Units.inchesToMeters(24.0)
                  && Math.abs(algaeIntakingError.getY()) <= Units.inchesToMeters(24.0))
              || (Math.abs(algaeIceCreamIntakingError.getX()) <= Units.inchesToMeters(12.0)
                  && Math.abs(algaeIceCreamIntakingError.getY()) <= Units.inchesToMeters(12.0)))) {
        hasAlgae = simIntakingTimerAlgae.hasElapsed(simIntakingTimeAlgae.get());
      } else {
        simIntakingTimerAlgae.restart();
      }
    }

    if (!isIntaking
        && (mailboxGoal != MailboxGoal.IDLE && mailboxGoal != MailboxGoal.CORALL4GRIP)) {
      hasCoral = false;
    }
    if (!isIntakingAlgae && (mailboxGoal != MailboxGoal.ALGAEHOLD)) {
      hasAlgae = false;
    }

    // Display hasCoral & hasAlgae
    SmartDashboard.putBoolean("Has Coral?", hasCoral);
    SmartDashboard.putBoolean("Has Algae?", hasAlgae);

    // Log state
    Logger.recordOutput("Manipulator/CoastOverride", coastOverride.getAsBoolean());
    Logger.recordOutput("Manipulator/DisabledOverride", disabledOverride.getAsBoolean());

    // Record cycle time
    LoggedTracer.record("Manipulator");
  }

  public void setGoal(Supplier<Rotation2d> goal) {
    this.goal = () -> goal.get().getRadians();
    atGoal = false;
  }

  public double getGoal() {
    return goal.getAsDouble();
  }

  @AutoLogOutput(key = "Manipulator/MeasuredExternalAngle")
  public Rotation2d getPivotAngle() {
    return pivotInputs.data.encoderAbsolutePosition();
  }

  @AutoLogOutput(key = "Manipulator/MeasuredInternalAngle")
  public Rotation2d getPivotAngleInternal() {
    return pivotInputs.data.internalPosition();
  }

  public void resetHasCoral(boolean value) {
    hasCoral = value;
    coralDebouncer = new Debouncer(coralDebounceTime, DebounceType.kRising);
    coralDebouncer.calculate(value);
  }

  public void resetHasAlgae(boolean value) {
    hasAlgae = value;
    algaeDebouncer = new Debouncer(algaeDebounceTime, DebounceType.kRising);
    algaeDebouncer.calculate(value);
  }

  public void setOverrides(BooleanSupplier coastOverride, BooleanSupplier disabledOverride) {
    this.coastOverride = coastOverride;
    this.disabledOverride = disabledOverride;
  }

  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    pivotIO.setBrakeMode(enabled);
  }

  public Command staticCharacterization() {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              stopProfile = true;
              timer.restart();
            },
            () -> {
              state.characterizationOutput = staticCharacterizationRampRate.get() * timer.get();
              pivotIO.runOpenLoop(state.characterizationOutput);
              Logger.recordOutput(
                  "Manipulator/StaticCharacterizationOutput", state.characterizationOutput);
            })
        .until(
            () ->
                pivotInputs.data.velocityRadPerSec() >= staticCharacterizationVelocityThresh.get())
        .andThen(pivotIO::stop)
        .andThen(Commands.idle())
        .finallyDo(
            () -> {
              stopProfile = false;
              timer.stop();
              Logger.recordOutput(
                  "Manipulator/CharacterizationOutput", state.characterizationOutput);
            });
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }
}
