// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.AutoFactory;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Choreographer;
import frc.robot.subsystems.Choreographer.WantedChoreography;
import frc.robot.subsystems.SubsystemVisualizer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.WantedState;
import frc.robot.subsystems.overridePublisher.OverridePublisher;
import frc.robot.subsystems.overridePublisher.OverridePublisherIO;
import frc.robot.subsystems.overridePublisher.OverridePublisherIOReal;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOTalonFX;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOSim;
import frc.robot.subsystems.rollers.RollerSystemIOSpark;
import frc.robot.subsystems.rollers.RollerSystemIOTalonFX;
import frc.robot.subsystems.sensors.CoralSensorIO;
import frc.robot.subsystems.sensors.CoralSensorIOColorSensor;
import frc.robot.subsystems.sensors.HomeSensorIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.testing.SubsystemTestMode;
import frc.robot.testing.TestDashboard;
import frc.robot.testing.TestManager;
import frc.robot.testing.testers.DriveTester;
import frc.robot.testing.testers.ElevatorTester;
import frc.robot.testing.testers.IntakeTester;
import frc.robot.testing.testers.ManualTuningTester;
import frc.robot.testing.testers.PivotTester;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AllianceUtil;
import frc.robot.util.Container;
import frc.robot.util.DoublePressTracker;
import frc.robot.util.TriggerUtil;
import java.util.Optional;
import java.util.Set;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@ExtensionMethod({DoublePressTracker.class, TriggerUtil.class})
public class RobotContainer {
  // Subsystems
  @Getter private Drive drive;
  @Getter private Elevator elevator;
  @Getter private Pivot wrist;
  @Getter private Intake intake;

  @Getter private Vision vision;
  @Getter private OverridePublisher overridePublisher;

  @Getter private SubsystemVisualizer subsystemVisualizerMeasured;
  @Getter private SubsystemVisualizer subsystemVisualizerGoal;

  @Getter private Choreographer choreographer;

  @Getter private AutoFactory autoFactory;

  // Testing system
  @Getter private TestManager testManager;
  @Getter private TestDashboard testDashboard;

  private final LoggedDashboardChooser<Pair<Pose2d, Command>> autoChooser =
      new LoggedDashboardChooser<>("Auto");

  // Controllers
  private static final CommandXboxController controller = new CommandXboxController(0);

  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      overridePublisher = new OverridePublisher(new OverridePublisherIOReal());

      switch (Constants.getRobot()) {
        case COMPBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
          elevator = new Elevator(new ElevatorIOTalonFX(), new HomeSensorIO() {});
          wrist = new Pivot(new PivotIOTalonFX());
          intake =
              new Intake(
                  new RollerSystemIOTalonFX(15, "rio", 40, false, false, 1),
                  new RollerSystemIOSpark(19, false),
                  new CoralSensorIOColorSensor(I2C.Port.kOnboard));
          break;
        }
        case DEVBOT -> {
          break;
        }
        case SIMBOT -> {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(TunerConstants.FrontLeft),
                  new ModuleIOSim(TunerConstants.FrontRight),
                  new ModuleIOSim(TunerConstants.BackLeft),
                  new ModuleIOSim(TunerConstants.BackRight));
          elevator = new Elevator(new ElevatorIOSim(), new HomeSensorIO() {});
          wrist = new Pivot(new PivotIOSim());
          intake =
              new Intake(
                  new RollerSystemIOSim(DCMotor.getKrakenX60(1), 1, 1),
                  new RollerSystemIOSim(DCMotor.getNeoVortex(1), 1, 1),
                  new CoralSensorIO() {});
          break;
        }
      }
    }

    // No-op implementations for replay or if not set above
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {}, new HomeSensorIO() {});
    }
    if (wrist == null) {
      wrist = new Pivot(new PivotIO() {});
    }
    if (intake == null) {
      intake = new Intake(new RollerSystemIO() {}, new RollerSystemIO() {}, new CoralSensorIO() {});
    }
    if (vision == null) {
      vision =
          new Vision(
              drive::addVisionMeasurement,
              cameras.values().stream().map(config -> new VisionIO() {}).toArray(VisionIO[]::new));
    }
    if (overridePublisher == null) {
      overridePublisher = new OverridePublisher(new OverridePublisherIO() {});
    }
    subsystemVisualizerMeasured =
        new SubsystemVisualizer(
            "Measured",
            elevator::getPosition,
            () -> wrist.getAngle().getRadians(),
            intake::hasCoral,
            intake::hasAlgae,
            drive::getPose);

    subsystemVisualizerGoal =
        new SubsystemVisualizer(
            "Goal",
            elevator::getGoalPosition,
            wrist.getGoalAngle()::getRadians,
            intake::hasCoral,
            intake::hasAlgae,
            drive::getPose);

    autoFactory = new AutoFactory(AllianceUtil.getAlliance(), this);

    choreographer =
        new Choreographer(drive, intake, elevator, wrist, overridePublisher, vision, controller);

    // Initialize testing system
    testManager = new TestManager();
    testDashboard = new TestDashboard(testManager);

    // Connect choreographer disable callback for testing
    testManager.setChoreographerDisableCallback(choreographer::setDisabled);

    // Register subsystem testers
    testManager.registerTester(SubsystemTestMode.DRIVE, new DriveTester(drive));
    testManager.registerTester(SubsystemTestMode.ELEVATOR, new ElevatorTester(elevator));
    testManager.registerTester(SubsystemTestMode.PIVOT, new PivotTester(wrist));
    testManager.registerTester(SubsystemTestMode.INTAKE, new IntakeTester(intake));
    testManager.registerTester(SubsystemTestMode.MANUAL, new ManualTuningTester(elevator, wrist));

    // Publish test instructions to dashboard
    testDashboard.publishInstructions();

    // Configure the button bindings
    configureButtonBindings();

    autoChooser.addDefaultOption("Taxi", autoFactory.createTaxiCommand());
    autoChooser.addOption("IKLJ", autoFactory.createIKLJAuto());
    autoChooser.addOption("EDC", autoFactory.createEDCAuto());
    autoChooser.addOption("FDCE", autoFactory.createFDCEAuto());
    autoChooser.addOption("Idle", autoFactory.createIdleCommand());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Coral score level selection
    final Container<Integer> selectedCoralScoreLevel = new Container<>(4);

    controller
        .povUp()
        .onTrue(
            Commands.runOnce(
                () -> {
                  selectedCoralScoreLevel.value = 4;
                }));
    controller
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  selectedCoralScoreLevel.value = 3;
                }));
    controller
        .povRight()
        .onTrue(
            Commands.runOnce(
                () -> {
                  selectedCoralScoreLevel.value = 2;
                }));
    controller
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  selectedCoralScoreLevel.value = 1;
                }));

    // Scoring side selection
    controller.x().onTrue(choreographer.flipScoringSideCommand());

    // Auto score
    controller
        .rightTrigger()
        .whileTrue(
            Commands.defer(
                    () -> {
                      return switch (selectedCoralScoreLevel.value) {
                        case 1 -> choreographer.setChoreographyCommand(WantedChoreography.SCORE_L1);
                        case 2 -> choreographer.setChoreographyCommand(WantedChoreography.SCORE_L2);
                        case 3 -> choreographer.setChoreographyCommand(WantedChoreography.SCORE_L3);
                        default ->
                            choreographer.setChoreographyCommand(WantedChoreography.SCORE_L4);
                      };
                    },
                    Set.of(choreographer))
                .withName("Auto Score Selected Level"))
        .onFalse(choreographer.setChoreographyCommand(WantedChoreography.DEFAULT_STATE));

    // Manual coral eject
    controller
        .b()
        .doublePress()
        .whileTrue(Commands.run(() -> intake.setWantedState(WantedState.OUTTAKE_CORAL)))
        .onFalse(
            Commands.runOnce(() -> intake.setWantedState(WantedState.OFF))
                .withName("Manual Coral Eject"));

    // Coral intake from station
    controller
        .leftTrigger()
        .whileTrue(
            choreographer
                .setChoreographyCommand(WantedChoreography.INTAKE_CORAL_FROM_STATION)
                .withName("Coral Station Intake"))
        .onFalse(choreographer.setChoreographyCommand(WantedChoreography.DEFAULT_STATE));

    // Algae triggers
    Trigger onOpposingSide =
        new Trigger(
            () ->
                AllianceFlipUtil.applyX(drive.getPose().getX()) > FieldConstants.FIELD_LENGTH / 2);

    Trigger shouldProcess =
        new Trigger(
            () ->
                AllianceFlipUtil.apply(
                                drive.getPose().exp(drive.getChassisSpeeds(true).toTwist2d(0.75)))
                            .getY()
                        < FieldConstants.FIELD_HEIGHT / 2 - Units.inchesToMeters(28)
                    || onOpposingSide.getAsBoolean());

    Container<Boolean> hasAlgae = new Container<>(false);
    controller.leftBumper().onTrue(Commands.runOnce(() -> hasAlgae.value = intake.hasAlgae()));

    // Algae reef intake
    controller
        .leftBumper()
        .and(() -> !hasAlgae.value)
        .whileTrue(
            choreographer
                .setChoreographyCommand(WantedChoreography.INTAKE_ALGAE_REEF)
                .withName("Algae Reef Intake"))
        .onFalse(choreographer.setChoreographyCommand(WantedChoreography.DEFAULT_STATE));

    // Algae pre-processor
    controller
        .leftBumper()
        .and(shouldProcess)
        .and(() -> hasAlgae.value)
        .and(controller.a().negate())
        .whileTrue(
            choreographer
                .setChoreographyCommand(WantedChoreography.MOVE_ALGAE_TO_PROCESSOR_POSITION)
                .withName("Algae Pre-Processor"))
        .onFalse(choreographer.setChoreographyCommand(WantedChoreography.DEFAULT_STATE));

    // Algae processor
    controller
        .leftBumper()
        .and(shouldProcess)
        .and(() -> hasAlgae.value)
        .and(controller.a())
        .whileTrue(
            choreographer
                .setChoreographyCommand(WantedChoreography.SCORE_ALGAE_IN_PROCESSOR)
                .withName("Algae Processing"))
        .onFalse(choreographer.setChoreographyCommand(WantedChoreography.DEFAULT_STATE));

    // Algae pre-net
    controller
        .leftBumper()
        .and(shouldProcess.negate())
        .and(() -> hasAlgae.value)
        .and(controller.a().negate())
        .whileTrue(
            choreographer
                .setChoreographyCommand(WantedChoreography.MOVE_ALGAE_TO_NET_POSITION)
                .withName("Algae Pre-Net"))
        .onFalse(choreographer.setChoreographyCommand(WantedChoreography.DEFAULT_STATE));

    // Algae net score
    controller
        .leftBumper()
        .and(shouldProcess.negate())
        .and(() -> hasAlgae.value)
        .and(controller.a())
        .whileTrue(
            choreographer
                .setChoreographyCommand(WantedChoreography.SCORE_ALGAE_IN_NET)
                .withName("Algae Net Score"))
        .onFalse(choreographer.setChoreographyCommand(WantedChoreography.DEFAULT_STATE));

    // Algae toss
    controller
        .a()
        .and(controller.leftBumper().negate())
        .whileTrue(
            choreographer
                .setChoreographyCommand(WantedChoreography.EJECT_ALGAE)
                .withName("Algae Toss"))
        .onFalse(choreographer.setChoreographyCommand(WantedChoreography.DEFAULT_STATE));

    // Reset gyro
    var driverStartAndBack = controller.start().and(controller.back());
    driverStartAndBack.onTrue(
        Commands.runOnce(
                () ->
                    drive.setPose(
                        new Pose2d(
                            drive.getPose().getX(),
                            drive.getPose().getY(),
                            Rotation2d.fromDegrees(AllianceUtil.isBlueAlliance() ? 0 : 180))))
            .withName("Reset Gyro")
            .ignoringDisable(true));

    // === TESTING SYSTEM CONTROLS ===
    // Note: These are typically used in test mode, not during competition
    // Start test: Hold Y + Start
    controller
        .y()
        .and(controller.start())
        .onTrue(testManager.startSelectedTest().withName("Start Selected Test"));

    // Stop test: Hold B + Start
    controller
        .b()
        .and(controller.start())
        .onTrue(testManager.stopTestCommand().withName("Stop Test"));

    // Endgame alerts
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(controllerRumbleCommand().withName("Controller Endgame Alert 1"));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            controllerRumbleCommand().withName("Controller Endgame Alert 2")); // Rumble three times
  }

  // Creates controller rumble command
  public static Command controllerRumbleCommand() {
    return Commands.startEnd(
            () -> {
              controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
            },
            () -> {
              controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            })
        .withName("Controller Rumble");
  }

  // Update dashboard data
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // Update test dashboard
    testDashboard.updateDashboard();
  }

  public void updateAlerts() {
    driverDisconnected.set(!DriverStation.isJoystickConnected(controller.getHID().getPort()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get().getSecond();
  }

  public Optional<Pose2d> getAutonomousStartingPose() {
    return Optional.of(autoChooser.get().getFirst());
  }
}
