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
import edu.wpi.first.math.system.plant.DCMotor;
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
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.commands.AlgaeScoreCommands;
import frc.robot.commands.AutoBuilder;
import frc.robot.commands.AutoScoreCommands;
import frc.robot.commands.DriveToStation;
import frc.robot.commands.IntakeCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.WantedState;
import frc.robot.subsystems.drive.SwerveIOCTRE;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOSim;
import frc.robot.subsystems.rollers.RollerSystemIOSpark;
import frc.robot.subsystems.rollers.RollerSystemIOTalonFX;
import frc.robot.subsystems.sensors.CoralSensorIO;
import frc.robot.subsystems.sensors.CoralSensorIOColorSensor;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.superstructure.manipulator.Manipulator;
import frc.robot.subsystems.superstructure.manipulator.PivotIO;
import frc.robot.subsystems.superstructure.manipulator.PivotIOSim;
import frc.robot.subsystems.superstructure.manipulator.PivotIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Container;
import frc.robot.util.DoublePressTracker;
import frc.robot.util.MirrorUtil;
import frc.robot.util.OverridePublisher;
import frc.robot.util.TriggerUtil;
import java.util.Optional;
import java.util.function.DoubleSupplier;
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
  private Vision vision;
  private final Superstructure superstructure;

  private Command blankAuto = Commands.none();

  // Controllers
  private static final CommandXboxController controller = new CommandXboxController(0);

  // Override controls for disabling auto-alignment features

  /** Disables auto-align when driving to coral station for intake */
  private final OverridePublisher disableCoralStationAutoAlign =
      new OverridePublisher("Disable Coral Station Auto Align");

  /** Disables auto-align for reef operations (coral scoring and algae intake) */
  private final OverridePublisher disableReefAutoAlign =
      new OverridePublisher("Disable Reef Auto Align");

  /** Disables auto-align for ice cream algae intake (typically used for testing) */
  private final OverridePublisher disableIceCreamAutoAlign =
      new OverridePublisher("Disable Ice Cream Auto Align");

  /** Disables auto-align for algae scoring operations (processing and net throwing) */
  private final OverridePublisher disableAlgaeScoreAutoAlign =
      new OverridePublisher("Disable Algae Score Auto Align");

  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert noAuto = new Alert("Please select an auto routine.", AlertType.kWarning);
  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
        moduleConstants = TunerConstants.getModuleConstants();

    drive =
        new Drive(
            new SwerveIOCTRE(
                TunerConstants.getSwerveDrivetrainConstants(), TunerConstants.getModuleConstants()),
            controller,
            moduleConstants[0].SpeedAt12Volts,
            moduleConstants[0].SpeedAt12Volts
                / Math.hypot(moduleConstants[0].LocationX, moduleConstants[0].LocationY));

    Elevator elevator = null;
    Manipulator manipulator = null;

    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case COMPBOT -> {
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  compCameras.values().stream()
                      .map(
                          config -> new VisionIOPhotonVision(config.name(), config.robotToCamera()))
                      .toArray(VisionIO[]::new));
          elevator = new Elevator(new ElevatorIOTalonFX());
          manipulator =
              new Manipulator(
                  new PivotIOTalonFX(),
                  new RollerSystemIOTalonFX(15, "", 40, false, true, 5),
                  new RollerSystemIOSpark(19, false),
                  new CoralSensorIOColorSensor(I2C.Port.kOnboard) {});
        }
        case DEVBOT -> {
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  devCameras.values().stream()
                      .map(
                          config ->
                              new VisionIOPhotonVisionSim(
                                  config.name(),
                                  config.robotToCamera(),
                                  RobotState.getInstance()::getRobotPoseFromSwerveDriveOdometry))
                      .toArray(VisionIO[]::new));
        }
        case SIMBOT -> {
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  simCameras.values().stream()
                      .map(
                          config ->
                              new VisionIOPhotonVisionSim(
                                  config.name(),
                                  config.robotToCamera(),
                                  RobotState.getInstance()::getRobotPoseFromSwerveDriveOdometry))
                      .toArray(VisionIO[]::new));
          elevator = new Elevator(new ElevatorIOSim());
          manipulator =
              new Manipulator(
                  new PivotIOSim(),
                  new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1, 1),
                  new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1, 1),
                  new CoralSensorIO() {});
        }
      }
    }

    // No-op implementations for replay or if not set above
    if (vision == null) {
      vision =
          new Vision(
              drive::addVisionMeasurement,
              cameras.values().stream().map(config -> new VisionIO() {}).toArray(VisionIO[]::new));
    }
    if (drive == null) {}
    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {});
    }
    if (manipulator == null) {
      manipulator =
          new Manipulator(
              new PivotIO() {},
              new RollerSystemIO() {},
              new RollerSystemIO() {},
              new CoralSensorIO() {});
    }
    superstructure = new Superstructure(elevator, manipulator);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    LoggedDashboardChooser<Boolean> mirror = new LoggedDashboardChooser<>("Processor Side?");
    mirror.addDefaultOption("Yes", false);
    mirror.addOption("No", true);
    LoggedDashboardChooser<Boolean> push = new LoggedDashboardChooser<>("Pushing?");
    push.addDefaultOption("No", false);
    push.addOption("Yes", true);
    MirrorUtil.setMirror(mirror::get);

    var autoBuilder = new AutoBuilder(drive, superstructure, push::get);
    autoChooser.addDefaultOption("None", blankAuto);
    autoChooser.addOption("Default Auto", autoBuilder.DefaultAuto());
    autoChooser.addOption("The One Piece is real!", autoBuilder.TheOnePiece());
    autoChooser.addOption("Taxi", autoBuilder.Taxi());

    /* // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)); */

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    DoubleSupplier driverX = () -> -controller.getLeftY();
    DoubleSupplier driverY = () -> -controller.getLeftX();
    DoubleSupplier driverOmega = () -> -controller.getRightX();

    final Container<ReefLevel> selectedCoralScoreLevel = new Container<>(ReefLevel.L4);

    // POV selection for coral score level
    controller.povUp().onTrue(Commands.runOnce(() -> selectedCoralScoreLevel.value = ReefLevel.L4));
    controller
        .povLeft()
        .onTrue(Commands.runOnce(() -> selectedCoralScoreLevel.value = ReefLevel.L3));
    controller
        .povRight()
        .onTrue(Commands.runOnce(() -> selectedCoralScoreLevel.value = ReefLevel.L2));
    controller
        .povDown()
        .onTrue(Commands.runOnce(() -> selectedCoralScoreLevel.value = ReefLevel.L1));

    // Auto score trigger (right trigger)
    Container<Boolean> autoScoreRunning = new Container<>(false);
    Trigger autoScoreAvailable = new Trigger(() -> drive.getClosestCoralObjective() != null);

    controller
        .rightTrigger()
        .and(autoScoreAvailable)
        .whileTrue(
            AutoScoreCommands.autoScore(
                    drive,
                    superstructure,
                    () -> selectedCoralScoreLevel.value,
                    () -> Optional.ofNullable(drive.getClosestCoralObjective()),
                    driverX,
                    driverY,
                    driverOmega,
                    Commands.none(),
                    controllerRumbleCommand(),
                    disableReefAutoAlign::getAsBoolean,
                    controller.b().doublePress()::getAsBoolean)
                .deadlineFor(
                    Commands.startEnd(
                        () -> autoScoreRunning.value = true, () -> autoScoreRunning.value = false))
                .withName("Auto Score Selected Level"))
        .onFalse(Commands.runOnce(() -> drive.setWantedState(WantedState.TELEOP_DRIVE)));
    controller
        .rightTrigger()
        .and(autoScoreAvailable.negate())
        .and(() -> !autoScoreRunning.value)
        .onTrue(
            Commands.sequence(
                controllerRumbleCommand().withTimeout(0.1),
                Commands.waitSeconds(0.1),
                controllerRumbleCommand().withTimeout(0.1)));
    // Super auto score (double press right trigger)
    Container<Boolean> superAutoScoreExecutionTracker = new Container<>(false);
    Container<ReefLevel> lockedSelectedCoralScoreLevelForSuper =
        new Container<>(
            selectedCoralScoreLevel.value); // Initialize with current, will be updated onTrue
    Container<FieldConstants.CoralObjective> lockedClosestCoralObjectiveForSuper =
        new Container<>(null);

    Trigger superAutoScoreCoralAvailable =
        new Trigger(() -> drive.getClosestCoralObjective() != null);
    Trigger canExecuteSuperAutoScore =
        superAutoScoreCoralAvailable.and(() -> !superstructure.hasAlgae());

    controller
        .rightTrigger()
        .doublePress()
        .and(canExecuteSuperAutoScore)
        .onTrue(
            Commands.runOnce(
                () -> {
                  lockedSelectedCoralScoreLevelForSuper.value = selectedCoralScoreLevel.value;
                  lockedClosestCoralObjectiveForSuper.value = drive.getClosestCoralObjective();
                }))
        .whileTrue(
            AutoScoreCommands.superAutoScore(
                    drive,
                    superstructure,
                    () -> lockedSelectedCoralScoreLevelForSuper.value,
                    () -> Optional.ofNullable(lockedClosestCoralObjectiveForSuper.value),
                    driverX,
                    driverY,
                    driverOmega,
                    () -> Commands.none(),
                    RobotContainer::controllerRumbleCommand,
                    disableReefAutoAlign::getAsBoolean,
                    controller.b().doublePress()::getAsBoolean)
                .deadlineFor(
                    Commands.startEnd(
                        () -> superAutoScoreExecutionTracker.value = true,
                        () -> superAutoScoreExecutionTracker.value = false))
                .withName("Super Auto Score"))
        .onFalse(Commands.runOnce(() -> drive.setWantedState(WantedState.TELEOP_DRIVE)));

    controller
        .rightTrigger()
        .doublePress()
        .and(canExecuteSuperAutoScore.negate())
        .and(() -> !superAutoScoreExecutionTracker.value)
        .onTrue(
            Commands.sequence(
                    controllerRumbleCommand().withTimeout(0.1),
                    Commands.waitSeconds(0.1),
                    controllerRumbleCommand().withTimeout(0.1))
                .withName("Super Auto Score Unavailable Rumble"));

    controller
        .rightBumper()
        .whileTrue(superstructure.backupCoral().withName("Manual Coral Backup"));

    controller
        .b()
        .doublePress()
        .and(controller.rightTrigger().negate())
        .whileTrue(superstructure.ejectCoral().withName("Manual Coral Eject"));

    // Coral intake
    controller
        .leftTrigger()
        .whileTrue(
            Commands.either(
                    Commands.none(), new DriveToStation(drive, false), disableCoralStationAutoAlign)
                .alongWith(
                    IntakeCommands.intake(superstructure),
                    Commands.runOnce(superstructure::resetHasCoral))
                .withName("Coral Station Intake"))
        .onFalse(
            Commands.runOnce(
                    () -> {
                      if (!superstructure.hasCoral()) {
                        superstructure.runGoal(() -> SuperstructureState.STOWREST).schedule();
                      }
                    })
                .alongWith(Commands.runOnce(() -> drive.setWantedState(WantedState.TELEOP_DRIVE))));

    // Algae reef intake & score
    Trigger onOpposingSide =
        new Trigger(
            () ->
                AllianceFlipUtil.applyX(
                        RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry().getX())
                    > FieldConstants.fieldLength / 2);
    Trigger shouldProcess =
        new Trigger(
            () ->
                AllianceFlipUtil.apply(
                                RobotState.getInstance()
                                    .getRobotPoseFromSwerveDriveOdometry()
                                    .exp(
                                        RobotState.getInstance()
                                            .getRobotChassisSpeeds()
                                            .toTwist2d(
                                                AlgaeScoreCommands.getLookaheadSecs().get())))
                            .getY()
                        < FieldConstants.fieldWidth / 2 - Drive.robotWidth
                    || onOpposingSide.getAsBoolean());
    Trigger shouldIceCream = new Trigger(() -> false);
    // new Trigger(() -> AllianceFlipUtil.applyX(drive.getPose().getX()) < 2.4);
    Container<Boolean> hasAlgae = new Container<>(false);
    controller
        .leftBumper()
        .onTrue(Commands.runOnce(() -> hasAlgae.value = superstructure.hasAlgae()));

    // Algae ice cream intake
    controller
        .leftBumper()
        .and(() -> !hasAlgae.value)
        .and(shouldIceCream)
        .whileTrue(
            AutoScoreCommands.iceCreamIntake(
                    drive,
                    superstructure,
                    () -> Optional.ofNullable(drive.getClosestIceCream()),
                    driverX,
                    driverY,
                    driverOmega,
                    Commands.none(),
                    disableIceCreamAutoAlign,
                    controller.a())
                .withName("Algae Ice Cream Intake"))
        .onFalse(Commands.runOnce(() -> drive.setWantedState(WantedState.TELEOP_DRIVE)));

    // Algae reef intake
    controller
        .leftBumper()
        .and(() -> !hasAlgae.value)
        .and(shouldIceCream.negate())
        .whileTrue(
            AutoScoreCommands.reefIntake(
                    drive,
                    superstructure,
                    () ->
                        !disableReefAutoAlign.getAsBoolean()
                            ? Optional.of(
                                new FieldConstants.AlgaeObjective(
                                    drive.getClosestCoralObjective().branchId() / 2))
                            : Optional.of(new FieldConstants.AlgaeObjective(1)),
                    driverX,
                    driverY,
                    driverOmega,
                    Commands.none(),
                    disableReefAutoAlign,
                    false)
                .withName("Algae Reef Intake"))
        .onFalse(Commands.runOnce(() -> drive.setWantedState(WantedState.TELEOP_DRIVE)));

    // Algae pre-processor
    controller
        .leftBumper()
        .and(shouldProcess)
        .and(() -> hasAlgae.value)
        .and(controller.a().negate())
        .or(AlgaeScoreCommands::shouldForceProcess)
        .whileTrue(
            AlgaeScoreCommands.process(
                    drive,
                    superstructure,
                    driverX,
                    driverY,
                    driverOmega,
                    () -> Commands.none(),
                    onOpposingSide,
                    controller.leftBumper(),
                    false,
                    disableAlgaeScoreAutoAlign)
                .withName("Algae Pre-Processor"))
        .onFalse(Commands.runOnce(() -> drive.setWantedState(WantedState.TELEOP_DRIVE)));

    // Algae process
    controller
        .leftBumper()
        .and(shouldProcess)
        .and(() -> hasAlgae.value)
        .and(controller.a())
        .whileTrue(
            AlgaeScoreCommands.process(
                    drive,
                    superstructure,
                    driverX,
                    driverY,
                    driverOmega,
                    () -> Commands.none(),
                    onOpposingSide,
                    controller.leftBumper(),
                    true,
                    disableAlgaeScoreAutoAlign)
                .withName("Algae Processing"))
        .onFalse(Commands.runOnce(() -> drive.setWantedState(WantedState.TELEOP_DRIVE)));

    // Algae pre-net
    controller
        .leftBumper()
        .and(shouldProcess.negate())
        .and(() -> hasAlgae.value)
        .and(controller.a().negate())
        .whileTrue(
            AlgaeScoreCommands.netThrowLineup(
                    drive,
                    superstructure,
                    driverX,
                    driverY,
                    Commands.none(),
                    disableAlgaeScoreAutoAlign)
                .withName("Algae Pre-Net"))
        // Indicate ready for score
        .and(() -> superstructure.getState() == SuperstructureState.PRE_THROW)
        .whileTrue(
            controllerRumbleCommand()
                .withTimeout(0.1)
                .andThen(Commands.waitSeconds(0.1))
                .repeatedly())
        .onFalse(Commands.runOnce(() -> drive.setWantedState(WantedState.TELEOP_DRIVE)));

    // Algae net score
    controller
        .leftBumper()
        .and(shouldProcess.negate())
        .and(() -> hasAlgae.value)
        .and(controller.a())
        .whileTrue(
            AlgaeScoreCommands.netThrowScore(drive, superstructure).withName("Algae Net Score"))
        .onFalse(Commands.runOnce(() -> drive.setWantedState(WantedState.TELEOP_DRIVE)));

    // Algae eject
    controller
        .a()
        .and(controller.leftBumper().negate())
        .whileTrue(superstructure.runGoal(SuperstructureState.TOSS).withName("Algae Toss"));

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
    // Controller disconnected alerts
    driverDisconnected.set(!DriverStation.isJoystickConnected(controller.getHID().getPort()));

    // Auto alert
    noAuto.set(
        DriverStation.isAutonomous()
            && !DriverStation.isEnabled()
            && autoChooser.get() == blankAuto);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
