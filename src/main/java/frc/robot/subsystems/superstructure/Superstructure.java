// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.FieldConstants;
import frc.robot.commands.DriveToStation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.SuperstructureStateData.Height;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.manipulator.Manipulator;
import frc.robot.subsystems.superstructure.manipulator.Manipulator.MailboxGoal;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTracer;
import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Builder;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Superstructure extends SubsystemBase {
  private static final Map<SuperstructureState, SuperstructureState> coralEjectPairs =
      Map.of(
          SuperstructureState.L1_CORAL,
          SuperstructureState.L1_CORAL_EJECT,
          SuperstructureState.L2_CORAL,
          SuperstructureState.L2_CORAL_EJECT,
          SuperstructureState.L3_CORAL,
          SuperstructureState.L3_CORAL_EJECT,
          SuperstructureState.L4_CORAL,
          SuperstructureState.L4_CORAL_EJECT);

  private final Elevator elevator;
  private final Manipulator manipulator;
  private final Drive drive;

  private final Graph<SuperstructureState, EdgeCommand> graph =
      new DefaultDirectedGraph<>(EdgeCommand.class);

  private EdgeCommand edgeCommand;

  @Getter private SuperstructureState state = SuperstructureState.START;
  private SuperstructureState next = null;
  @Getter private SuperstructureState goal = SuperstructureState.START;
  private boolean wasDisabled = false;

  @AutoLogOutput(key = "Superstructure/EStopped")
  private boolean isEStopped = false;

  private LoggedNetworkBoolean characterizationModeOn =
      new LoggedNetworkBoolean("/SmartDashboard/Characterization Mode On", false);

  private BooleanSupplier disableOverride = () -> false;
  private final Alert driverDisableAlert =
      new Alert("Superstructure disabled due to driver override.", Alert.AlertType.kWarning);
  private final Alert emergencyDisableAlert =
      new Alert(
          "Superstructure emergency disabled due to high position error. Disable the superstructure manually and reenable to reset.",
          Alert.AlertType.kError);

  private SuperstructureVisualizer measuredVisualizer;
  private SuperstructureVisualizer setpointVisualizer;
  private SuperstructureVisualizer goalVisualizer;

  @Setter private Optional<SuperstructureState> reefDangerState = Optional.empty();

  public Superstructure(Elevator elevator, Manipulator manipulator, Drive drive) {
    this.elevator = elevator;
    this.manipulator = manipulator;
    this.drive = drive;

    this.measuredVisualizer = new SuperstructureVisualizer("Measured");
    this.setpointVisualizer = new SuperstructureVisualizer("Setpoint");
    this.goalVisualizer = new SuperstructureVisualizer("Goal");

    // Updating E Stop based on disabled override
    new Trigger(() -> disableOverride.getAsBoolean())
        .onFalse(Commands.runOnce(() -> isEStopped = false).ignoringDisable(true));

    // Add states as vertices
    for (var state : SuperstructureState.values()) {
      graph.addVertex(state);
    }

    // Populate edges
    // Add edge from start to stow
    graph.addEdge(
        SuperstructureState.START,
        SuperstructureState.STOWTRAVEL,
        EdgeCommand.builder()
            .command(
                runSuperstructureExtras(SuperstructureState.STOWTRAVEL)
                    .andThen(
                        runManipulatorPivot(
                            () ->
                                SuperstructureState.STOWTRAVEL
                                    .getValue()
                                    .getPose()
                                    .pivotAngle()
                                    .get()),
                        Commands.parallel(
                            Commands.waitSeconds(0.2).andThen(elevator.homingSequence())),
                        runSuperstructurePose(SuperstructureState.STOWTRAVEL.getValue().getPose()),
                        Commands.waitUntil(this::mechanismsAtGoal)))
            .build());

    graph.addEdge(
        SuperstructureState.AUTO_START,
        SuperstructureState.STOWTRAVEL,
        EdgeCommand.builder()
            .command(
                runSuperstructureExtras(SuperstructureState.STOWTRAVEL)
                    .andThen(
                        runManipulatorPivot(
                            () ->
                                SuperstructureState.STOWTRAVEL
                                    .getValue()
                                    .getPose()
                                    .pivotAngle()
                                    .get()),
                        Commands.parallel(
                            Commands.waitSeconds(0.2).andThen(elevator.homingSequence())),
                        runSuperstructurePose(SuperstructureState.STOWTRAVEL.getValue().getPose()),
                        Commands.waitUntil(this::mechanismsAtGoal)))
            .build());

    graph.addEdge(
        SuperstructureState.SAFETY,
        SuperstructureState.STOWTRAVEL,
        EdgeCommand.builder()
            .command(
                runManipulatorPivot(
                        () ->
                            Rotation2d.fromRadians(
                                MathUtil.clamp(
                                    manipulator.getPivotAngle().getRadians(),
                                    pivotMinSafeAngleRad.get(),
                                    pivotMaxSafeAngleRad.get())))
                    .andThen(
                        runSuperstructureExtras(SuperstructureState.STOWTRAVEL),
                        Commands.waitUntil(manipulator::isAtGoal),
                        runSuperstructurePose(SuperstructureState.STOWTRAVEL.getValue().getPose()),
                        Commands.waitUntil(this::mechanismsAtGoal)))
            .build());

    graph.addEdge(
        SuperstructureState.L1_CORAL_EJECT,
        SuperstructureState.STOWTRAVEL,
        EdgeCommand.builder()
            .command(
                runSuperstructureExtras(SuperstructureState.STOWTRAVEL)
                    .andThen(
                        runManipulatorPivot(
                                () ->
                                    Rotation2d.fromRadians(
                                        MathUtil.clamp(
                                            SuperstructureState.STOWTRAVEL
                                                .getValue()
                                                .getPose()
                                                .pivotAngle()
                                                .get()
                                                .getRadians(),
                                            pivotMinSafeAngleRad.get(),
                                            pivotMaxSafeAngleRad.get())))
                            .andThen(
                                Commands.waitUntil(this::mechanismsAtGoal),
                                runSuperstructurePose(
                                    SuperstructureState.STOWTRAVEL.getValue().getPose())),
                        Commands.waitUntil(this::mechanismsAtGoal)))
            .build());

    graph.addEdge(
        SuperstructureState.STOWTRAVEL,
        SuperstructureState.L1_CORAL,
        EdgeCommand.builder()
            .command(
                runElevator(
                        () ->
                            SuperstructureState.L1_CORAL
                                .getValue()
                                .getPose()
                                .elevatorHeight()
                                .getAsDouble())
                    .andThen(
                        Commands.waitUntil(elevator::isAtGoal),
                        runManipulatorPivot(
                            () ->
                                SuperstructureState.L1_CORAL
                                    .getValue()
                                    .getPose()
                                    .pivotAngle()
                                    .get()),
                        Commands.waitUntil(this::mechanismsAtGoal),
                        runSuperstructureExtras(SuperstructureState.L1_CORAL)))
            .build());

    graph.addEdge(
        SuperstructureState.PRE_THROW,
        SuperstructureState.THROW,
        EdgeCommand.builder()
            .command(
                runSuperstructurePose(SuperstructureState.THROW.getValue().getPose())
                    .andThen(
                        Commands.waitUntil(
                            () -> {
                              double interpolationFactor = 0.9;
                              double preThrowPivotAngleRad =
                                  SuperstructureState.PRE_THROW
                                      .getValue()
                                      .getPose()
                                      .pivotAngle()
                                      .get()
                                      .getRadians();
                              double throwPivotAngleRad =
                                  SuperstructureState.THROW
                                      .getValue()
                                      .getPose()
                                      .pivotAngle()
                                      .get()
                                      .getRadians();
                              double releaseThresholdAngleRad =
                                  MathUtil.interpolate(
                                      throwPivotAngleRad,
                                      preThrowPivotAngleRad,
                                      interpolationFactor);
                              return Math.abs(manipulator.getPivotAngle().getRadians())
                                  <= Math.abs(releaseThresholdAngleRad);
                            }),
                        runSuperstructureExtras(SuperstructureState.THROW),
                        Commands.waitUntil(this::mechanismsAtGoal)))
            .build());

    graph.addEdge(
        SuperstructureState.CHARACTERIZATION,
        SuperstructureState.STOWTRAVEL,
        EdgeCommand.builder()
            .command(Commands.idle(this).until(() -> !characterizationModeOn.get()))
            .build());

    final Set<SuperstructureState> dangerStates =
        Set.of(SuperstructureState.CORAL_INTAKE, SuperstructureState.STOWREST);

    final Set<SuperstructureState> freeNoAlgaeStates =
        Set.of(
            SuperstructureState.STOWREST,
            SuperstructureState.STOWTRAVEL,
            SuperstructureState.L1_CORAL,
            SuperstructureState.L2_CORAL,
            SuperstructureState.L3_CORAL,
            SuperstructureState.L4_CORAL,
            SuperstructureState.ALGAE_L2_INTAKE,
            SuperstructureState.ALGAE_L3_INTAKE,
            SuperstructureState.ALGAE_ICE_CREAM_INTAKE);

    final Set<SuperstructureState> freeAlgaeStates =
        Set.of(
            SuperstructureState.ALGAE_STOW,
            SuperstructureState.ALGAE_ICE_CREAM_INTAKE,
            SuperstructureState.ALGAE_L2_INTAKE,
            SuperstructureState.ALGAE_L3_INTAKE,
            SuperstructureState.PRE_THROW);

    final Set<SuperstructureState> algaeIntakeStates =
        Set.of(
            SuperstructureState.ALGAE_L2_INTAKE,
            SuperstructureState.ALGAE_L3_INTAKE,
            SuperstructureState.ALGAE_ICE_CREAM_INTAKE);

    // Add edges from states in which arm could get crushed
    for (var from : dangerStates) {
      for (var state : SuperstructureState.values()) {
        if (from == state) continue;
        if (dangerStates.contains(state)) continue;
        if (state == SuperstructureState.L1_CORAL) continue;
        graph.addEdge(
            from,
            state,
            EdgeCommand.builder()
                .command(
                    runManipulatorPivot(
                            () ->
                                Rotation2d.fromRadians(
                                    MathUtil.clamp(
                                        state.getValue().getPose().pivotAngle().get().getRadians(),
                                        pivotMinSafeAngleRad.get(),
                                        pivotMaxSafeAngleRad.get())))
                        .andThen(
                            Commands.waitUntil(manipulator::isAtGoal),
                            runSuperstructurePose(state.getValue().getPose()),
                            Commands.waitUntil(this::mechanismsAtGoal),
                            runSuperstructureExtras(state)))
                .algaeEdgeType(AlgaeEdge.NONE)
                .build());
      }
    }

    // Add all free edges
    for (var from : freeNoAlgaeStates) {
      for (var to : freeNoAlgaeStates) {
        if (from == to) continue;
        if (algaeIntakeStates.contains(from) && algaeIntakeStates.contains(to)) continue;
        if (algaeIntakeStates.contains(from)) {
          addEdge(from, to, AlgaeEdge.NO_ALGAE);
        } else {
          addEdge(from, to);
        }
      }
    }

    for (var from : freeAlgaeStates) {
      for (var to : freeAlgaeStates) {
        if (from == to) continue;
        if (algaeIntakeStates.contains(from) && algaeIntakeStates.contains(to)) continue;
        if (algaeIntakeStates.contains(from)) {
          addEdge(from, to, AlgaeEdge.ALGAE);
        } else {
          addEdge(from, to);
        }
      }
    }

    for (var from : algaeIntakeStates) {
      for (var to : algaeIntakeStates) {
        if (from == to) continue;
        addEdge(from, to);
      }
    }

    // Add edges for paired states
    final Set<Pair<SuperstructureState, SuperstructureState>> pairedStates =
        Set.of(
            Pair.of(SuperstructureState.STOWTRAVEL, SuperstructureState.CORAL_INTAKE),
            Pair.of(SuperstructureState.STOWREST, SuperstructureState.CORAL_INTAKE),
            Pair.of(SuperstructureState.L1_CORAL, SuperstructureState.L1_CORAL_EJECT),
            Pair.of(SuperstructureState.L2_CORAL, SuperstructureState.L2_CORAL_EJECT),
            Pair.of(SuperstructureState.L3_CORAL, SuperstructureState.L3_CORAL_EJECT),
            Pair.of(SuperstructureState.L4_CORAL, SuperstructureState.L4_CORAL_EJECT),
            Pair.of(SuperstructureState.PRE_PROCESS, SuperstructureState.PROCESS),
            Pair.of(SuperstructureState.PRE_THROW, SuperstructureState.THROW),
            Pair.of(SuperstructureState.ALGAE_STOW, SuperstructureState.TOSS));
    for (var pair : pairedStates) {
      addEdge(pair.getFirst(), pair.getSecond(), true, AlgaeEdge.NONE, false);
    }

    // Add recoverable algae states
    for (var from :
        Set.of(
            SuperstructureState.ALGAE_STOW,
            SuperstructureState.PRE_PROCESS,
            SuperstructureState.PRE_THROW,
            SuperstructureState.PROCESS,
            SuperstructureState.TOSS,
            SuperstructureState.THROW)) {
      for (var to : freeNoAlgaeStates) {
        addEdge(from, to, AlgaeEdge.NO_ALGAE);
      }
    }

    // Add miscellaneous edges
    addEdge(
        SuperstructureState.STOWTRAVEL,
        SuperstructureState.ALGAE_STOW,
        false,
        AlgaeEdge.ALGAE,
        false);
    addEdge(
        SuperstructureState.STOWREST,
        SuperstructureState.ALGAE_STOW,
        false,
        AlgaeEdge.ALGAE,
        false);
    addEdge(
        SuperstructureState.ALGAE_STOW,
        SuperstructureState.STOWTRAVEL,
        false,
        AlgaeEdge.NO_ALGAE,
        false);
    addEdge(
        SuperstructureState.ALGAE_STOW,
        SuperstructureState.PRE_PROCESS,
        true,
        AlgaeEdge.NONE,
        false);

    setDefaultCommand(
        runGoal(
            (Supplier<SuperstructureState>)
                () -> {
                  final Pose2d robot = drive.getPose();
                  final Pose2d flippedRobot = AllianceFlipUtil.apply(robot);

                  // Check danger state
                  if (reefDangerState.isPresent()
                      && DriveToStation.withinDistanceToReef(robot, Units.inchesToMeters(4.0))
                      && Math.abs(
                              FieldConstants.Reef.center
                                  .minus(flippedRobot.getTranslation())
                                  .getAngle()
                                  .minus(flippedRobot.getRotation())
                                  .getDegrees())
                          <= 30) {
                    // Reset reef danger state when new goal requested
                    if (goal != reefDangerState.get()
                        && !(coralEjectPairs.containsKey(reefDangerState.get())
                            && goal == coralEjectPairs.get(reefDangerState.get()))) {
                      reefDangerState = Optional.empty();
                    } else {
                      return reefDangerState.get();
                    }
                  } else if (reefDangerState.isPresent()) {
                    reefDangerState = Optional.empty();
                  }

                  return manipulator.hasAlgae()
                      ? SuperstructureState.ALGAE_STOW
                      : SuperstructureState.STOWTRAVEL;
                }));
  }

  @Override
  public void periodic() {
    // Run periodic
    elevator.periodic();
    manipulator.periodic();

    if (characterizationModeOn.get()) {
      state = SuperstructureState.CHARACTERIZATION;
      next = null;
    } else {
    }

    if ((DriverStation.isDisabled() || isEStopped) && !wasDisabled && elevator.isHomed()) {
      state = SuperstructureState.SAFETY;
    }
    wasDisabled = DriverStation.isDisabled() || isEStopped;

    if (DriverStation.isDisabled()) {
      next = null;
    } else if (edgeCommand == null || !edgeCommand.getCommand().isScheduled()) {
      // Update edge to new state
      if (next != null) {
        state = next;
        next = null;
      }

      // Schedule next command in sequence
      if (state != goal) {
        bfs(state, goal)
            .ifPresent(
                next -> {
                  this.next = next;
                  edgeCommand = graph.getEdge(state, next);
                  edgeCommand.getCommand().schedule();
                });
      }
    }

    // Tell elevator we are stowed
    elevator.setStowed(
        (state == SuperstructureState.STOWTRAVEL && goal == SuperstructureState.STOWTRAVEL)
            || (state == SuperstructureState.ALGAE_STOW && goal == SuperstructureState.ALGAE_STOW));

    // Tell elevator if we have algae
    elevator.setHasAlgae(manipulator.hasAlgae());

    // Tell manipulator if intaking
    manipulator.setIntaking(
        state == SuperstructureState.CORAL_INTAKE || next == SuperstructureState.CORAL_INTAKE);

    manipulator.setIntakingAlgae(
        state == SuperstructureState.ALGAE_L2_INTAKE
            || next == SuperstructureState.ALGAE_L2_INTAKE
            || state == SuperstructureState.ALGAE_L3_INTAKE
            || next == SuperstructureState.ALGAE_L3_INTAKE
            || state == SuperstructureState.ALGAE_ICE_CREAM_INTAKE
            || next == SuperstructureState.ALGAE_ICE_CREAM_INTAKE);

    // E Stop Manipulator and Elevator if Necessary
    isEStopped =
        (isEStopped || elevator.isShouldEStop() || manipulator.isShouldEStop())
            && Constants.getMode() == Mode.REAL;
    elevator.setEStopped(isEStopped);
    manipulator.setEStopped(isEStopped);

    driverDisableAlert.set(disableOverride.getAsBoolean());
    emergencyDisableAlert.set(isEStopped);

    // Log state
    Logger.recordOutput("Superstructure/State", state);
    Logger.recordOutput("Superstructure/Next", next);
    Logger.recordOutput("Superstructure/Goal", goal);
    if (edgeCommand != null) {
      Logger.recordOutput(
          "Superstructure/EdgeCommand",
          graph.getEdgeSource(edgeCommand) + " --> " + graph.getEdgeTarget(edgeCommand));
    } else {
      Logger.recordOutput("Superstructure/EdgeCommand", "");
    }
    Logger.recordOutput(
        "Superstructure/ReefDangerState",
        reefDangerState.map(SuperstructureState::toString).orElse(""));

    // Update visualizer
    measuredVisualizer.update(
        elevator.getPositionMeters(),
        manipulator.getPivotAngle().getRadians(),
        hasCoral(),
        hasAlgae(),
        drive.getPose());
    setpointVisualizer.update(
        elevator.getPositionMeters(),
        manipulator.getPivotAngle().getRadians(),
        hasCoral(),
        hasAlgae(),
        drive.getPose());
    goalVisualizer.update(
        elevator.getPositionMeters(),
        manipulator.getPivotAngle().getRadians(),
        hasCoral(),
        hasAlgae(),
        drive.getPose());

    // Record cycle time
    LoggedTracer.record("Superstructure");
  }

  public void setOverrides(BooleanSupplier disableOverride) {
    this.disableOverride = disableOverride;
  }

  @AutoLogOutput(key = "Superstructure/AtGoal")
  public boolean atGoal() {
    return state == goal;
  }

  public boolean hasAlgae() {
    return manipulator.hasAlgae();
  }

  public boolean hasCoral() {
    return manipulator.hasCoral();
  }

  public boolean readyForL4() {
    return elevator.getPositionMeters() >= elevatorL4ClearHeight.get();
  }

  public void resetHasCoral() {
    manipulator.resetHasCoral(false);
  }

  public void resetHasAlgae() {
    manipulator.resetHasAlgae(false);
  }

  private void setGoal(SuperstructureState goal) {
    // Don't do anything if goal is the same
    if (this.goal == goal) return;
    this.goal = goal;

    if (next == null) return;

    var edgeToCurrentState = graph.getEdge(next, state);
    // Figure out if we should schedule a different command to get to goal faster
    if (edgeCommand.getCommand().isScheduled()
        && edgeToCurrentState != null
        && isEdgeAllowed(edgeToCurrentState, goal)) {
      // Figure out where we would have gone from the previous state
      bfs(state, goal)
          .ifPresent(
              newNext -> {
                if (newNext == next) {
                  // We are already on track
                  return;
                }

                if (newNext != state && graph.getEdge(next, newNext) != null) {
                  // We can skip directly to the newNext edge
                  edgeCommand.getCommand().cancel();
                  edgeCommand = graph.getEdge(state, newNext);
                  edgeCommand.getCommand().schedule();
                  next = newNext;
                } else {
                  // Follow the reverse edge from next back to the current edge
                  edgeCommand.getCommand().cancel();
                  edgeCommand = graph.getEdge(next, state);
                  edgeCommand.getCommand().schedule();
                  var temp = state;
                  state = next;
                  next = temp;
                }
              });
    }
  }

  public Command runGoal(SuperstructureState goal) {
    return runOnce(() -> setGoal(goal)).andThen(Commands.idle(this));
  }

  public Command runGoal(Supplier<SuperstructureState> goal) {
    return run(() -> setGoal(goal.get()));
  }

  public Command forceEjectManipulator() {
    return startEnd(
        () -> manipulator.setMailboxGoal(MailboxGoal.CORALEJECT),
        () -> manipulator.setMailboxGoal(MailboxGoal.IDLE));
  }

  public void setAutoStart() {
    state = SuperstructureState.AUTO_START;
    next = null;
    if (edgeCommand != null) {
      edgeCommand.getCommand().cancel();
    }
    manipulator.resetHasCoral(true);
  }

  private Optional<SuperstructureState> bfs(SuperstructureState start, SuperstructureState goal) {
    // Map to track the parent of each visited node
    Map<SuperstructureState, SuperstructureState> parents = new HashMap<>();
    Queue<SuperstructureState> queue = new LinkedList<>();
    queue.add(start);
    parents.put(start, null); // Mark the start node as visited with no parent
    // Perform BFS
    while (!queue.isEmpty()) {
      SuperstructureState current = queue.poll();
      // Check if we've reached the goal
      if (current == goal) {
        break;
      }
      // Process valid neighbors
      for (EdgeCommand edge :
          graph.outgoingEdgesOf(current).stream()
              .filter(edge -> isEdgeAllowed(edge, goal))
              .toList()) {
        SuperstructureState neighbor = graph.getEdgeTarget(edge);
        // Only process unvisited neighbors
        if (!parents.containsKey(neighbor)) {
          parents.put(neighbor, current);
          queue.add(neighbor);
        }
      }
    }

    // Reconstruct the path to the goal if found
    if (!parents.containsKey(goal)) {
      return Optional.empty(); // Goal not reachable
    }

    // Trace back the path from goal to start
    SuperstructureState nextState = goal;
    while (!nextState.equals(start)) {
      SuperstructureState parent = parents.get(nextState);
      if (parent == null) {
        return Optional.empty(); // No valid path found
      } else if (parent.equals(start)) {
        // Return the edge from start to the next node
        return Optional.of(nextState);
      }
      nextState = parent;
    }
    return Optional.of(nextState);
  }

  private void addEdge(SuperstructureState from, SuperstructureState to) {
    addEdge(from, to, AlgaeEdge.NONE);
  }

  private void addEdge(SuperstructureState from, SuperstructureState to, AlgaeEdge algaeEdge) {
    addEdge(from, to, false, algaeEdge, false);
  }

  private void addEdge(
      SuperstructureState from,
      SuperstructureState to,
      boolean reverse,
      AlgaeEdge algaeEdge,
      boolean restricted) {
    graph.addEdge(
        from,
        to,
        EdgeCommand.builder()
            .command(getEdgeCommand(from, to))
            .algaeEdgeType(algaeEdge)
            .restricted(restricted)
            .build());
    if (reverse) {
      graph.addEdge(
          to,
          from,
          EdgeCommand.builder()
              .command(getEdgeCommand(to, from))
              .algaeEdgeType(algaeEdge)
              .restricted(restricted)
              .build());
    }
  }

  private Command getEdgeCommand(SuperstructureState from, SuperstructureState to) {
    boolean passesThroughCrossMember =
        from.getValue().getHeight().equals(Height.BOTTOM)
            != to.getValue().getHeight().equals(Height.BOTTOM);
    if (passesThroughCrossMember) {
      if (to.getValue().getPose().elevatorHeight().getAsDouble() > 0.05) {
        return runManipulatorPivot(
                () ->
                    Rotation2d.fromRadians(
                        MathUtil.clamp(
                            to.getValue().getPose().pivotAngle().get().getRadians(),
                            pivotMinSafeAngleRad.get(),
                            pivotMaxSafeAngleRad.get())))
            .andThen(
                Commands.waitUntil(manipulator::isAtGoal),
                runSuperstructurePose(to.getValue().getPose()),
                Commands.waitUntil(this::mechanismsAtGoal))
            .andThen(runSuperstructureExtras(to));
      }
    }
    return runSuperstructurePose(to.getValue().getPose())
        .andThen(Commands.waitUntil(this::mechanismsAtGoal))
        .deadlineFor(runSuperstructureExtras(to));
  }

  public Command runHomingSequence() {
    return runOnce(
        () -> {
          state = SuperstructureState.START;
          next = null;
          if (edgeCommand != null) {
            edgeCommand.command.cancel();
          }
        });
  }

  public Command setCharacterizationMode() {
    return runOnce(
        () -> {
          state = SuperstructureState.CHARACTERIZATION;
          characterizationModeOn.set(true);
          next = null;
          if (edgeCommand != null) {
            edgeCommand.getCommand().cancel();
          }
        });
  }

  public Command runElevator(DoubleSupplier elevatorHeight) {
    return Commands.runOnce(() -> elevator.setGoal(elevatorHeight));
  }

  public Command runManipulatorPivot(Supplier<Rotation2d> pivotAngle) {
    return Commands.runOnce(() -> manipulator.setGoal(pivotAngle));
  }

  /** Runs elevator and pivot to {@link SuperstructurePose} pose. Ends immediately. */
  private Command runSuperstructurePose(SuperstructurePose pose) {
    return runElevator(pose.elevatorHeight()).alongWith(runManipulatorPivot(pose.pivotAngle()));
  }

  /** Runs manipulator and slam based on {@link SuperstructureState} state. Ends immediately. */
  private Command runSuperstructureExtras(SuperstructureState state) {
    return Commands.runOnce(
        () -> {
          manipulator.setMailboxGoal(state.getValue().getMailboxGoal());
        });
  }

  private boolean isEdgeAllowed(EdgeCommand edge, SuperstructureState goal) {
    return (!edge.isRestricted() || goal == graph.getEdgeTarget(edge))
        && (edge.getAlgaeEdgeType() == AlgaeEdge.NONE
            || manipulator.hasAlgae() == (edge.getAlgaeEdgeType() == AlgaeEdge.ALGAE));
  }

  private boolean mechanismsAtGoal() {
    return elevator.isAtGoal()
        && (manipulator.isAtGoal() || Constants.getRobot() == RobotType.DEVBOT);
  }

  /** Get coral scoring state for level and algae state */
  public static SuperstructureState getScoringState(
      FieldConstants.ReefLevel height, boolean eject) {
    var stateGroup = CoralScoreStateGroup.valueOf(height.toString());
    return eject ? stateGroup.getEjectState() : stateGroup.getHoldState();
  }

  @RequiredArgsConstructor
  @Getter
  private enum CoralScoreStateGroup {
    L1(SuperstructureState.L1_CORAL, SuperstructureState.L1_CORAL_EJECT),
    L2(SuperstructureState.L2_CORAL, SuperstructureState.L2_CORAL_EJECT),
    L3(SuperstructureState.L3_CORAL, SuperstructureState.L3_CORAL_EJECT),
    L4(SuperstructureState.L4_CORAL, SuperstructureState.L4_CORAL_EJECT);

    private final SuperstructureState holdState;
    private final SuperstructureState ejectState;
  }

  /** All edge commands should finish and exit properly. */
  @Builder(toBuilder = true)
  @Getter
  public static class EdgeCommand extends DefaultEdge {
    private final Command command;
    @Builder.Default private final boolean restricted = false;
    @Builder.Default private final AlgaeEdge algaeEdgeType = AlgaeEdge.NONE;
  }

  private enum AlgaeEdge {
    NONE,
    NO_ALGAE,
    ALGAE
  }
}
