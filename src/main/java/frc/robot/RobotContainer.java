// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlgaeScoreCommands;
import frc.robot.commands.AutoScoreCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToStation;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
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
import frc.robot.util.TriggerUtil;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
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
  private Drive drive;
  private Vision vision;
  private final Superstructure superstructure;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Trigger disableReefAutoAlign = new Trigger(() -> false);
  private final Trigger disableCoralStationAutoAlign = new Trigger(() -> false);
  private final Trigger disableAlgaeScoreAutoAlign = new Trigger(() -> false);

  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Elevator elevator = null;
    Manipulator manipulator = null;

    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case COMPBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVision(camera0Name, robotToCamera0),
                  new VisionIOPhotonVision(camera1Name, robotToCamera1));
          elevator = new Elevator(new ElevatorIOTalonFX());
          manipulator =
              new Manipulator(
                  new PivotIOTalonFX(),
                  new RollerSystemIOTalonFX(15, "", 40, false, true, 5),
                  new RollerSystemIOSpark(19, false),
                  new CoralSensorIOColorSensor(I2C.Port.kOnboard) {},
                  drive);
        }
        case DEVBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVision(camera0Name, robotToCamera0));
        }
        case SIMBOT -> {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(TunerConstants.FrontLeft),
                  new ModuleIOSim(TunerConstants.FrontRight),
                  new ModuleIOSim(TunerConstants.BackLeft),
                  new ModuleIOSim(TunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                  new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
          elevator = new Elevator(new ElevatorIOSim());
          manipulator =
              new Manipulator(
                  new PivotIOSim(),
                  new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1, 1),
                  new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1, 1),
                  new CoralSensorIO() {},
                  drive);
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
    if (vision == null) {
      switch (Constants.getRobot()) {
        case COMPBOT ->
            vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        case DEVBOT -> vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        default -> vision = new Vision(drive::addVisionMeasurement);
      }
    }
    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {});
    }
    if (manipulator == null) {
      manipulator =
          new Manipulator(
              new PivotIO() {},
              new RollerSystemIO() {},
              new RollerSystemIO() {},
              new CoralSensorIO() {},
              drive);
    }
    superstructure = new Superstructure(elevator, manipulator, drive);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
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
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  // Moved leftCoral to be a field so it can be modified in lambdas
  @AutoLogOutput private int leftCoral = 0;

  private void configureButtonBindings() {
    DoubleSupplier driverX = () -> -controller.getLeftY();
    DoubleSupplier driverY = () -> -controller.getLeftX();
    DoubleSupplier driverOmega = () -> -controller.getRightX();

    Supplier<Command> joystickDriveCommandFactory =
        () -> DriveCommands.joystickDrive(drive, driverX, driverY, driverOmega);
    drive.setDefaultCommand(joystickDriveCommandFactory.get());

    // Coral intake
    controller
        .leftTrigger()
        .whileTrue(
            Commands.either(
                    joystickDriveCommandFactory.get(),
                    new DriveToStation(drive, driverX, driverY, driverOmega, false),
                    disableCoralStationAutoAlign)
                .alongWith(
                    // IntakeCommands.intake(superstructure, funnel),
                    Commands.runOnce(superstructure::resetHasCoral))
                .withName("Coral Station Intake"));

    // Algae reef intake & score
    Trigger onOpposingSide =
        new Trigger(
            () -> AllianceFlipUtil.applyX(drive.getPose().getX()) > FieldConstants.fieldLength / 2);
    Trigger shouldProcess =
        new Trigger(
            () ->
                AllianceFlipUtil.apply(
                                drive
                                    .getPose()
                                    .exp(
                                        drive
                                            .getChassisSpeeds()
                                            .toTwist2d(
                                                AlgaeScoreCommands.getLookaheadSecs().get())))
                            .getY()
                        < FieldConstants.fieldWidth / 2 - Drive.DRIVE_BASE_WIDTH
                    || onOpposingSide.getAsBoolean());
    Container<Boolean> hasAlgae = new Container<>(false);
    controller
        .leftBumper()
        .onTrue(Commands.runOnce(() -> hasAlgae.value = superstructure.hasAlgae()));

    // Algae reef intake
    // TODO: Use a fixed AlgaeObjective for reef intake
    controller
        .leftBumper()
        .and(() -> !hasAlgae.value)
        .whileTrue(
            AutoScoreCommands.reefIntake(
                    drive,
                    superstructure,
                    () -> Optional.of(new frc.robot.FieldConstants.AlgaeObjective(0)),
                    driverX,
                    driverY,
                    driverOmega,
                    joystickDriveCommandFactory.get(),
                    () -> false,
                    disableReefAutoAlign)
                .withName("Algae Reef Intake (Test)"));

    // Algae pre-processor
    controller
        .leftBumper()
        .and(shouldProcess)
        .and(() -> hasAlgae.value)
        .and(controller.a().negate())
        .or(AlgaeScoreCommands::shouldForceProcess)
        .whileTrueContinuous(
            AlgaeScoreCommands.process(
                    drive,
                    superstructure,
                    driverX,
                    driverY,
                    driverOmega,
                    joystickDriveCommandFactory,
                    onOpposingSide,
                    controller.leftBumper(),
                    false,
                    disableAlgaeScoreAutoAlign)
                .withName("Algae Pre-Processor"));

    // Algae process
    controller
        .leftBumper()
        .and(shouldProcess)
        .and(() -> hasAlgae.value)
        .and(controller.a())
        .whileTrueContinuous(
            AlgaeScoreCommands.process(
                    drive,
                    superstructure,
                    driverX,
                    driverY,
                    driverOmega,
                    joystickDriveCommandFactory,
                    onOpposingSide,
                    controller.leftBumper(),
                    true,
                    disableAlgaeScoreAutoAlign)
                .withName("Algae Processing"));

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
                    joystickDriveCommandFactory.get(),
                    disableAlgaeScoreAutoAlign)
                .withName("Algae Pre-Net"))
        // Indicate ready for score
        .and(() -> superstructure.getState() == SuperstructureState.PRE_THROW)
        .whileTrue(
            controllerRumbleCommand()
                .withTimeout(0.1)
                .andThen(Commands.waitSeconds(0.1))
                .repeatedly());

    // Algae net score
    controller
        .leftBumper()
        .and(shouldProcess.negate())
        .and(() -> hasAlgae.value)
        .and(controller.a())
        .whileTrue(
            AlgaeScoreCommands.netThrowScore(drive, superstructure).withName("Algae Net Score"));

    // Algae eject
    controller
        .a()
        .and(controller.leftBumper().negate())
        .whileTrue(superstructure.runGoal(SuperstructureState.TOSS).withName("Algae Toss"));

    // Reset gyro
    var driverStartAndBack = controller.start().and(controller.back());
    driverStartAndBack.onTrue(
        Commands.runOnce(
                () ->
                    drive.setPose(
                        new Pose2d(
                            drive.getPose().getTranslation(),
                            AllianceFlipUtil.apply(Rotation2d.kZero))))
            .withName("Reset Gyro")
            .ignoringDisable(true));

    // Raise elevator
    controller
        .povUp()
        .toggleOnTrue(
            superstructure.runElevator(() -> .6).withName("Force Raise Elevator l2 algae"))
        .toggleOnFalse(superstructure.runElevator(() -> 0.25));

    controller
        .povLeft()
        .toggleOnTrue(
            superstructure.runElevator(() -> 1.0668).withName("Force Raise Elevator l3 algae"))
        .toggleOnFalse(superstructure.runElevator(() -> 0.25));

    controller
        .povRight()
        .toggleOnTrue(superstructure.runElevator(() -> 1.6).withName("Force Raise Elevator"))
        .toggleOnFalse(superstructure.runElevator(() -> 0.25));

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
  private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }

  // Update dashboard data
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public void updateAlerts() {
    // Controller disconnected alerts
    driverDisconnected.set(!DriverStation.isJoystickConnected(controller.getHID().getPort()));
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
