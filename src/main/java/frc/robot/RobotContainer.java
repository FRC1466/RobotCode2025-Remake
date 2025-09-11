// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Choreographer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOSim;
import frc.robot.util.AllianceUtil;
import frc.robot.util.DoublePressTracker;
import frc.robot.util.TriggerUtil;
import java.util.Optional;
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
  @Getter private Intake intake;

  @Getter private Choreographer choreographer;

  @Getter private AutoFactory autoFactory;

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

      switch (Constants.getRobot()) {
        case COMPBOT -> {
          break;
        }
        case DEVBOT -> {
          break;
        }
        case SIMBOT -> {
          intake = new Intake(new RollerSystemIOSim(DCMotor.getFalcon500(1), 3, 0.01));
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
    if (intake == null) {
      intake = new Intake(new RollerSystemIO() {});
    }

    choreographer = new Choreographer(drive, intake);

    autoFactory = new AutoFactory(AllianceUtil.getAlliance(), this);

    autoChooser.addDefaultOption("Idle Auto", autoFactory.createIdleCommand());
    autoChooser.addOption("Taxi Auto", autoFactory.createTaxiCommand());
    autoChooser.addOption("Taxi then L1 Auto", autoFactory.createTaxiThenScoreL1());

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
    // Simple intake / outtake bindings
    controller
        .leftTrigger()
        .whileTrue(choreographer.intakeFromStationCommand().withName("Intake"))
        .onFalse(choreographer.defaultStateCommand());

    controller
        .rightTrigger()
        .whileTrue(choreographer.scoreL1Command().withName("Outtake"))
        .onFalse(choreographer.defaultStateCommand());

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
