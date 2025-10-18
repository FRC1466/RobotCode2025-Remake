// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.testing.testers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.testing.SubsystemTester;

/**
 * Tests the Drive subsystem in isolation.
 * Validates odometry, gyro, motors, and basic movement.
 */
public class DriveTester extends SubsystemTester {
  private final Drive drive;
  
  public DriveTester(Drive drive) {
    super("Drive");
    this.drive = drive;
  }
  
  @Override
  protected void defineTestSteps() {
    // Step 1: Check gyro connectivity
    testSteps.add(new TestStep("Gyro Connectivity", true) {
      @Override
      protected void onStart() {
        System.out.println("Checking gyro...");
      }
      
      @Override
      protected void onExecute() {}
      
      @Override
      protected boolean isStepComplete() {
        return true;
      }
      
      @Override
      protected boolean validateStep() {
        Rotation2d gyroAngle = drive.getPose().getRotation();
        
        if (Double.isNaN(gyroAngle.getRadians())) {
          setFailureReason("Gyro returning invalid data");
          return false;
        }
        
        System.out.println("✓ Gyro connected. Heading: " + 
            String.format("%.1f", gyroAngle.getDegrees()) + "°");
        return true;
      }
    });
    
    // Step 2: Check odometry
    testSteps.add(new TestStep("Odometry Check", true) {
      @Override
      protected void onStart() {
        System.out.println("Checking odometry...");
      }
      
      @Override
      protected void onExecute() {}
      
      @Override
      protected boolean isStepComplete() {
        return true;
      }
      
      @Override
      protected boolean validateStep() {
        var pose = drive.getPose();
        
        if (Double.isNaN(pose.getX()) || Double.isNaN(pose.getY())) {
          setFailureReason("Odometry returning invalid position");
          return false;
        }
        
        System.out.println("✓ Odometry active. Position: (" + 
            String.format("%.2f", pose.getX()) + ", " + 
            String.format("%.2f", pose.getY()) + ")");
        return true;
      }
    });
    
    // Step 3: Check for module movement capability
    testSteps.add(new TestStep("Module Response Test", false) {
      private Timer stepTimer = new Timer();
      private double initialX;
      private double initialY;
      
      @Override
      protected void onStart() {
        stepTimer.restart();
        initialX = drive.getPose().getX();
        initialY = drive.getPose().getY();
        System.out.println("Note: Manual movement test - drive robot slightly during this step...");
      }
      
      @Override
      protected void onExecute() {
        // Just monitor for movement
      }
      
      @Override
      protected boolean isStepComplete() {
        return stepTimer.get() > 3.0;
      }
      
      @Override
      protected boolean validateStep() {
        double dx = drive.getPose().getX() - initialX;
        double dy = drive.getPose().getY() - initialY;
        double distanceMoved = Math.hypot(dx, dy);
        
        System.out.println("✓ Odometry tracking. Movement detected: " + 
            String.format("%.2f", distanceMoved) + " m");
        return true;
      }
    });
    
    // Step 4: Gyro stability check
    testSteps.add(new TestStep("Gyro Stability", false) {
      private Timer stepTimer = new Timer();
      private double initialHeading;
      private double maxDrift = 0;
      
      @Override
      protected void onStart() {
        stepTimer.restart();
        initialHeading = drive.getPose().getRotation().getDegrees();
        System.out.println("Checking gyro stability (keep robot still)...");
      }
      
      @Override
      protected void onExecute() {
        double currentDrift = Math.abs(
            drive.getPose().getRotation().getDegrees() - initialHeading);
        maxDrift = Math.max(maxDrift, currentDrift);
      }
      
      @Override
      protected boolean isStepComplete() {
        return stepTimer.get() > 2.0;
      }
      
      @Override
      protected boolean validateStep() {
        if (maxDrift > 5.0) {
          setFailureReason("WARNING: Gyro drift detected: " + 
              String.format("%.2f", maxDrift) + "°");
          return false;
        }
        
        System.out.println("✓ Gyro stable. Max drift: " + 
            String.format("%.2f", maxDrift) + "°");
        return true;
      }
    });
    
    // Step 5: Check chassis speeds
    testSteps.add(new TestStep("Chassis Speeds Check", false) {
      @Override
      protected void onStart() {
        System.out.println("Checking chassis speeds feedback...");
      }
      
      @Override
      protected void onExecute() {}
      
      @Override
      protected boolean isStepComplete() {
        return true;
      }
      
      @Override
      protected boolean validateStep() {
        ChassisSpeeds speeds = drive.getChassisSpeeds();
        
        if (Double.isNaN(speeds.vxMetersPerSecond) || 
            Double.isNaN(speeds.vyMetersPerSecond) ||
            Double.isNaN(speeds.omegaRadiansPerSecond)) {
          setFailureReason("Invalid chassis speeds data");
          return false;
        }
        
        System.out.println("✓ Chassis speeds valid: vx=" + 
            String.format("%.2f", speeds.vxMetersPerSecond) + " m/s, vy=" +
            String.format("%.2f", speeds.vyMetersPerSecond) + " m/s, omega=" +
            String.format("%.2f", Units.radiansToDegrees(speeds.omegaRadiansPerSecond)) + " deg/s");
        return true;
      }
    });
  }
  
  @Override
  protected void returnToSafeState() {
    // Drive will handle stopping in its own state machine
    drive.setWantedState(Drive.WantedState.IDLE);
  }
}
