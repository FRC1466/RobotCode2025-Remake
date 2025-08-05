// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * A {@link edu.wpi.first.wpilibj.smartdashboard.SendableChooser} for selecting an autonomous
 * program. <br>
 * <br>
 *
 * <p>How to add a new autonomous program.
 *
 * <ol>
 *   <li>Add a new value to {@link Auto}<br>
 *       The name of the value should use screaming snake case
 *   <li>Add a new method in {@link AutoFactory} that returns a {@link
 *       edu.wpi.first.wpilibj2.command.Command}.
 *   <li>Add a new {@link AutoProgram} to {@link AutoChooser#AUTO_PROGRAMS}.
 *   <li>Implement the autonomous program factory method.
 *   <li>Test, test, and test some more.
 * </ol>
 */
public class AutoChooser extends SendableChooser<Auto> {

  private static final List<AutoProgram> AUTO_PROGRAMS =
      List.of(
          new AutoProgram(Auto.IDLE, "IDLE", AutoFactory::createIdleCommand),
          new AutoProgram(Auto.TAXI, "TAXI", AutoFactory::createTaxiCommand),
          new AutoProgram(Auto.EDC, "3x Processor, EDC", AutoFactory::createEDCAuto),
          new AutoProgram(Auto.JKL, "3x Opposite, JKL", AutoFactory::createIdleCommand),
          new AutoProgram(Auto.IKLJ, "4x Opposite, IKLJ", AutoFactory::createIKLJAuto),
          new AutoProgram(Auto.FDCE, "4x Processor, FDCE", AutoFactory::createFDCEAuto));

  /**
   * Create a new <code>AutoChooser</code>
   *
   * @param robotContainer A {@link RobotContainer}
   * @return A new <code>AutoChooser</code> populated with the programs defined in the private field
   *     {@link AutoChooser#AUTO_PROGRAMS}.
   */
  public static AutoChooser create(final RobotContainer robotContainer) {
    var autoFactories =
        Stream.of(DriverStation.Alliance.values())
            .map(alliance -> Map.entry(alliance, new AutoFactory(alliance, robotContainer)))
            .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));
    var programs =
        AUTO_PROGRAMS.stream()
            .map(program -> Map.entry(program.getAuto(), program))
            .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));

    var autoChooser = new AutoChooser(programs, autoFactories);

    AUTO_PROGRAMS.forEach(
        program -> {
          if (program.getAuto() == Auto.IDLE) {
            autoChooser.setDefaultOption(program.getLabel(), program.getAuto());
          } else {
            autoChooser.addOption(program.getLabel(), program.getAuto());
          }
        });

    autoChooser.reset(null);

    return autoChooser;
  }

  /**
   * Update the <code>AutoChooser</code><br>
   * <br>
   *
   * <p>The commands for the selected autonomous program are loaded and cached (if not already
   * present).
   */
  public void update() {
    var selected = getSelected();

    Stream.of(DriverStation.Alliance.values())
        .forEach(
            alliance -> {
              commandCache
                  .get(alliance)
                  .computeIfAbsent(
                      selected,
                      auto ->
                          Pair.of(loadStartingPose(alliance, auto), loadCommand(alliance, auto)));
            });
  }

  /**
   * Reset the caches behind this <code>AutoChooser</code>
   *
   * @param key Optional {@link edu.wpi.first.networktables.NetworkTable} key. If provided the
   *     selected value of the entry at this location will also be reset.
   */
  public void reset(final String key) {
    Stream.of(DriverStation.Alliance.values())
        .forEach(alliance -> commandCache.get(alliance).clear());

    if (key != null) {
      var table = NetworkTableInstance.getDefault().getTable(key);
      table.putValue("selected", NetworkTableValue.makeString("%s".formatted(Auto.IDLE)));
    }
  }

  /**
   * Get the {@link Command} for the selected autonomous program, if available.
   *
   * @return The {@link Command} for the selected autonomous program as an {@link Optional} if
   *     available, otherwise {@link Optional#empty}.
   */
  public Optional<Command> getSelectedCommand() {
    var selected = getSelected();

    return DriverStation.getAlliance()
        .map(
            alliance -> {
              System.out.printf("Running program %s/%s\n", alliance, selected);

              return commandCache.get(alliance).get(selected).getSecond();
            });
  }

  public Optional<Pose2d> getStartingPose() {
    var selected = getSelected();

    return DriverStation.getAlliance()
        .map(
            alliance -> {
              return commandCache.get(alliance).get(selected).getFirst();
            });
  }

  public AutoProgram getProgram() {
    return programs.get(getSelected());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.publishConstString("selected", "%s".formatted(Auto.IDLE));
  }

  private Command loadCommand(final DriverStation.Alliance alliance, final Auto auto) {
    var program = programs.get(auto);

    System.out.printf("Loading command %s/%s\n", alliance, auto);

    return program.getCommand(autoFactories.get(alliance));
  }

  private Pose2d loadStartingPose(final DriverStation.Alliance alliance, final Auto auto) {
    var program = programs.get(auto);

    System.out.printf("Loading command %s/%s\n", alliance, auto);

    return program.getStartingPose(autoFactories.get(alliance));
  }

  private final Map<Auto, AutoProgram> programs;

  private final Map<DriverStation.Alliance, Map<Auto, Pair<Pose2d, Command>>> commandCache;

  private final Map<DriverStation.Alliance, AutoFactory> autoFactories;

  private AutoChooser(
      final Map<Auto, AutoProgram> programs,
      final Map<DriverStation.Alliance, AutoFactory> autoFactories) {
    this.programs = programs;
    this.autoFactories = autoFactories;
    commandCache =
        Stream.of(DriverStation.Alliance.values())
            .map(alliance -> Map.entry(alliance, new HashMap<Auto, Pair<Pose2d, Command>>()))
            .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));
  }
}
