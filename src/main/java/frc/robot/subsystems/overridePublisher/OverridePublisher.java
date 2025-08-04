// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.overridePublisher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/**
 * Manages multiple boolean overrides through SmartDashboard with AdvantageKit logging. Uses Lombok
 * to generate typed getter methods for each override.
 */
public class OverridePublisher extends SubsystemBase {
  private final OverridePublisherIO io;
  private final OverridePublisherIOInputsAutoLogged inputs =
      new OverridePublisherIOInputsAutoLogged();
  private final String logKey = "OverridePublisher";

  // Lombok will generate getters for these fields
  @Getter private boolean OverrideAll;
  @Getter private boolean ReefOverride;
  @Getter private boolean AlgaeOverride;
  @Getter private boolean StationOverride;

  public OverridePublisher(OverridePublisherIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(logKey, inputs);

    // Update the fields from inputs
    OverrideAll = inputs.OverrideAll;
    ReefOverride = inputs.ReefOverride;
    AlgaeOverride = inputs.AlgaeOverride;
    StationOverride = inputs.StationOverride;
  }
}
