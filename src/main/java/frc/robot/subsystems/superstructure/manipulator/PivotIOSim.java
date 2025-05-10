// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.manipulator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.SuperstructureConstants;

public class PivotIOSim implements PivotIO {
  // Hardware simulation objects
  private final DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
  private final double armLength = Units.inchesToMeters(10.0); // Example length
  private final double minAngle = Manipulator.minAngle.getRadians();
  private final double maxAngle = Manipulator.maxAngle.getRadians();
  private final double moi = (1.0 / 3.0) * (frc.robot.subsystems.superstructure.elevator.ElevatorIOSim.carriageMassKg);

  private final Encoder encoder = new Encoder(27, 28);
  private final PWMSparkMax motor = new PWMSparkMax(19);
  private final EncoderSim encoderSim = new EncoderSim(encoder);

  private final SingleJointedArmSim armSim = new SingleJointedArmSim(
      gearbox,
      200, // gear ratio
      moi,
      armLength,
      minAngle,
      maxAngle,
      true,
      SuperstructureConstants.elevatorAngle.getRadians(),
      2.0 * Math.PI / 4096,
      0.0 // No noise
  );

  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private double feedforward = 0.0;
  private boolean closedLoop = false;

  public PivotIOSim() {
    encoder.setDistancePerPulse(2.0 * Math.PI / 4096);
    encoderSim.setDistance(Manipulator.maxAngle.getRadians() - 0.1);
    armSim.setState(Manipulator.maxAngle.getRadians() - 0.1, 0.0);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (!closedLoop) {
      controller.reset();
      // Open loop: set input voltage directly
      armSim.setInput(motor.get() * RobotController.getBatteryVoltage());
    } else {
      // Closed loop: PID output as voltage
      double pidOutput = controller.calculate(encoder.getDistance());
      armSim.setInput(pidOutput + feedforward);
    }

    // Simulate physics for 20ms
    armSim.update(Constants.loopPeriodSecs);

    // Update simulated encoder
    encoderSim.setDistance(armSim.getAngleRads());

    // Output data
    inputs.data = new PivotIOData(
        true,
        true,
        Rotation2d.fromRadians(armSim.getAngleRads()),
        armSim.getVelocityRadPerSec(),
        armSim.getInput(0), // last applied voltage
        armSim.getCurrentDrawAmps(),
        0.0, // inputTorqueCurrent not directly available
        0.0
    );
  }

  @Override
  public void runOpenLoop(double output) {
    closedLoop = false;
    motor.set(output);
  }

  @Override
  public void runVolts(double volts) {
    closedLoop = false;
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    runOpenLoop(0.0);
  }

  @Override
  public void runPosition(Rotation2d position, double feedforward) {
    closedLoop = true;
    controller.setSetpoint(position.getRadians());
    this.feedforward = feedforward;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }
}
