// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.AutoFactory;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Choreographer;
import frc.robot.subsystems.Choreographer.WantedChoreography;
import frc.robot.subsystems.MechanismVisualizer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.WantedState;
import frc.robot.subsystems.overridePublisher.OverridePublisher;
import frc.robot.subsystems.overridePublisher.OverridePublisherIO;
import frc.robot.subsystems.overridePublisher.OverridePublisherIOReal;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOSim;
import frc.robot.subsystems.sensors.CoralSensorIO;
import frc.robot.subsystems.sensors.HomeSensorIO;
import frc.robot.subsystems.slapdown.Slapdown;
import frc.robot.subsystems.slapdown.SlapdownIO;
import frc.robot.subsystems.slapdown.SlapdownIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.util.AllianceFlipUtil;
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
  @Getter private Wrist wrist;
  @Getter private Slapdown slapdown;
  @Getter private Intake intake;
  @Getter private Vision vision;
  @Getter private OverridePublisher overridePublisher;
  @Getter private MechanismVisualizer mechanismVisualizer;

  @Getter private Choreographer choreographer;

  @Getter
  private AutoFactory autoFactory =
      new AutoFactory(DriverStation.getAlliance().orElse(Alliance.Blue), this);

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
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
        moduleConstants = TunerConstants.getModuleConstants();

    if (Constants.getMode() != Constants.Mode.REPLAY) {
      drive =
          new Drive(
              new DriveIOCTRE(
                  TunerConstants.getSwerveDrivetrainConstants(),
                  TunerConstants.getModuleConstants()),
              controller,
              moduleConstants[0].SpeedAt12Volts,
              moduleConstants[0].SpeedAt12Volts
                  / Math.hypot(moduleConstants[0].LocationX, moduleConstants[0].LocationY));

      overridePublisher = new OverridePublisher(new OverridePublisherIOReal());

      switch (Constants.getRobot()) {
        case COMPBOT -> {
          break;
        }
        case DEVBOT -> {
          break;
        }
        case SIMBOT -> {
          elevator = new Elevator(new ElevatorIOSim(), new HomeSensorIO() {});
          wrist = new Wrist(new WristIOSim(), elevator::getPosition);
          slapdown = new Slapdown(new SlapdownIOSim());
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
              new DriveIO() {},
              controller,
              moduleConstants[0].SpeedAt12Volts,
              moduleConstants[0].SpeedAt12Volts
                  / Math.hypot(moduleConstants[0].LocationX, moduleConstants[0].LocationY));
    }
    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {}, new HomeSensorIO() {});
    }
    if (wrist == null) {
      wrist = new Wrist(new WristIO() {}, elevator::getPosition);
    }
    if (slapdown == null) {
      slapdown = new Slapdown(new SlapdownIO() {});
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
    final var table = NetworkTableInstance.getDefault().getTable("AdvantageScope");
    mechanismVisualizer = new MechanismVisualizer(table);

    choreographer =
        new Choreographer(drive, intake, elevator, wrist, slapdown, overridePublisher, vision);

    mechanismVisualizer.setStageTravel(new double[] {0.3, 0.297078, 0.297078, 0.296373});
    mechanismVisualizer.setStageZeroOffsets(new double[] {0.0, 0.0, 0.0, 0.0});

    // Configure the button bindings
    configureButtonBindings();

    autoChooser.addDefaultOption("Idle", autoFactory.createIdleCommand());
    autoChooser.addOption("Taxi", autoFactory.createTaxiCommand());
    autoChooser.addOption("3x Processor Side", autoFactory.createEDCAuto());
    autoChooser.addOption("4x Processor Side", autoFactory.createFDCEAuto());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Coral score level selection
    final Container<Integer> selectedCoralScoreLevel = new Container<>(4);

    controller.povUp().onTrue(Commands.runOnce(() -> selectedCoralScoreLevel.value = 4));
    controller.povLeft().onTrue(Commands.runOnce(() -> selectedCoralScoreLevel.value = 3));
    controller.povRight().onTrue(Commands.runOnce(() -> selectedCoralScoreLevel.value = 2));
    controller.povDown().onTrue(Commands.runOnce(() -> selectedCoralScoreLevel.value = 1));

    // Scoring side selection
    controller.x().onTrue(choreographer.flipScoringSideCommand());

    // Auto score
    controller
        .rightTrigger()
        .whileTrue(
            Commands.defer(
                    () -> {
                      return switch (selectedCoralScoreLevel.value) {
                        case 1 ->
                            choreographer.setChoreographyCommand(WantedChoreography.SCORE_L1);
                        case 2 ->
                            choreographer.setChoreographyCommand(WantedChoreography.SCORE_L2);
                        case 3 ->
                            choreographer.setChoreographyCommand(WantedChoreography.SCORE_L3);
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

    // Coral intake from ground
    controller
        .leftTrigger()
        .whileTrue(
            choreographer
                .setChoreographyCommand(WantedChoreography.INTAKE_CORAL_FROM_GROUND)
                .withName("Coral Ground Intake"))
        .onFalse(choreographer.setChoreographyCommand(WantedChoreography.DEFAULT_STATE));

    // Algae triggers
    Trigger onOpposingSide =
        new Trigger(
            () ->
                AllianceFlipUtil.applyX(
                        RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry().getX())
                    > FieldConstants.FIELD_LENGTH / 2);

    Trigger shouldProcess =
        new Trigger(
            () ->
                AllianceFlipUtil.apply(
                                RobotState.getInstance()
                                    .getRobotPoseFromSwerveDriveOdometry()
                                    .exp(
                                        RobotState.getInstance()
                                            .getRobotChassisSpeeds()
                                            .toTwist2d(0.75)))
                            .getY()
                        < FieldConstants.FIELD_HEIGHT / 2 - Drive.robotWidth
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
        Commands.runOnce(() -> drive.resetRotationBasedOnAlliance())
            .withName("Reset Gyro")
            .ignoringDisable(true));

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
