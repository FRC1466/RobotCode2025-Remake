# Subsystem Testing System

## Overview

The Subsystem Testing System is a comprehensive framework designed to help identify and resolve sim-to-real issues by allowing isolated testing and validation of individual robot subsystems.

## Problem It Solves

After programming robots that work perfectly in simulation, teams often encounter numerous small issues on the real robot. These issues are difficult to diagnose because:
- Multiple subsystems interact simultaneously
- It's hard to isolate which subsystem is causing problems
- Sensor readings may differ between sim and real hardware
- Motor behavior may vary between simulation and reality

## Solution

This testing system allows you to:
1. **Isolate subsystems** - Test one subsystem at a time
2. **Disable interference** - Automatically disables Choreographer during tests
3. **Validate functionality** - Run automated test sequences
4. **Manual setpoint tuning** - Create and test positions via NetworkTables
5. **Track health** - Get clear pass/fail indicators for each component
6. **Identify issues** - Detailed logging shows exactly what failed and why
7. **Run repeatedly** - Tests can be run multiple times without restarting robot

---

## Available Test Modes

### Automated Testing Modes
- **Drive Test** - Tests gyro, odometry, module response, stability
- **Elevator Test** - Tests sensors, position control, stability
- **Pivot Test** - Tests angle sensor, position control to multiple angles
- **Intake Test** - Tests rollers, sensors, grip, outtake
- **All Subsystems** - Runs all tests sequentially
- **None** - Normal robot operation (default)

### Manual Testing Mode
- **Manual Tuning** - Control elevator and pivot positions directly via NetworkTables
  - Perfect for creating and testing setpoints
  - Works in both simulation and on real robot
  - See [MANUAL_TUNING.md](MANUAL_TUNING.md) for detailed guide

---

## Quick Start Guide

### 1. Setup

The testing system is already integrated into your robot code. No additional setup required!

### 2. Safety First

⚠️ **IMPORTANT**: Before running tests:
- Place robot on blocks (wheels off ground)
- Clear the area around the robot
- Have emergency stop ready
- Start with Drive subsystem disabled or secured

### 3. Running a Test

**Via Shuffleboard/Dashboard:**
1. Open Shuffleboard and find the "Subsystem Test Mode" chooser
2. Select the subsystem you want to test (e.g., "Elevator")
3. Enable the robot in TEST mode
4. Hold **Y + Start** buttons to start the test
   - ⚠️ **Choreographer is automatically disabled** when test starts
5. Watch the console and dashboard for test progress
6. To stop: Hold **B + Start** or disable the robot
   - ✅ **Choreographer is automatically re-enabled** when test stops

**Via SmartDashboard:**
- Same process, look for "Test Mode" chooser and status widgets

### 4. Reading Results

**Console Output:**
```
⚠️ CHOREOGRAPHER DISABLED FOR TESTING
=== Starting Elevator Test ===
Checking elevator sensors...
✓ Sensors connected. Position: 0.50 in, Velocity: 0.00 in/s
✓ Motor current monitoring available
Moving to mid position: 12.0 inches
✓ Reached mid position in 2.34 seconds
...
=== Elevator Test Complete ===
Result: PASSED
✅ CHOREOGRAPHER RE-ENABLED
```

**Dashboard Display:**
- **Test/CurrentMode**: Shows which test is running
- **Test/Active**: Boolean indicator
- **Test/Progress**: Percentage complete
- **Test/Health**: Overall health status (✅ Pass, ⚠️ Warning, ❌ Fail)
- **Test/Summary**: Quick overview of results

**AdvantageScope:**
All test data is logged under `Testing/` in AdvantageKit logs:
- `Testing/[Subsystem]/TestState`
- `Testing/[Subsystem]/Health/*`
- `Testing/[Subsystem]/CurrentStep`

---

## Available Tests

### Drive Subsystem Test
**Tests:**
- Gyro connectivity and stability
- Odometry accuracy
- Chassis speeds feedback
- Manual movement tracking

**What to Watch:**
- Gyro angle should be stable when robot is still
- Position should track when you manually push robot
- No NaN or infinite values in readings

**Common Issues:**
- Gyro drift: Check IMU connection and calibration
- Odometry not updating: Check encoder connections
- Invalid readings: Verify CAN bus health

---

### Elevator Subsystem Test
**Tests:**
- Sensor connectivity (position, velocity)
- Motor current monitoring
- Position control accuracy
- Holding stability
- Return to home position

**What to Watch:**
- Position readings should be reasonable (not extreme values)
- Elevator should reach target positions smoothly
- No excessive drift when holding position
- Current draw should be reasonable

**Common Issues:**
- Position sensor invalid: Check encoder wiring
- Can't reach position: Check PID tuning or mechanical binding
- High current: Check for mechanical friction or damaged motors
- Position drift: Check if brake mode is enabled

---

### Pivot/Wrist Subsystem Test
**Tests:**
- Angle sensor connectivity
- Position control to multiple angles
- Return to home position

**What to Watch:**
- Angle readings should be valid
- Smooth movement between positions
- Reaches target angles accurately

**Common Issues:**
- Angle sensor issues: Verify absolute encoder connection
- Can't reach angles: Check mechanical range of motion and PID
- Oscillation: PID tuning needed

---

### Intake Subsystem Test
**Tests:**
- End effector motor operation
- Star wheel motor operation
- Coral sensor detection
- Grip function
- Outtake function

**What to Watch:**
- Both motors should spin without errors
- Sensor should detect game pieces when present
- All states should execute without faults

**Common Issues:**
- Motor not spinning: Check motor controller CAN ID and wiring
- Sensor not detecting: Verify sensor type and threshold settings
- Unexpected behavior: Check state machine logic

---

## Interpreting Test Results

### Health Status Icons
- ✅ **PASS**: Test step completed successfully
- ⚠️ **WARNING**: Issue detected but not critical
- ❌ **FAIL**: Critical issue, test stopped
- ⚪ **UNKNOWN**: Test not yet run

### Understanding Failures

When a test fails, the console will show:
```
❌ Test FAILED at step: Position Control Test (Mid)
   Reason: Failed to reach target position within timeout.
   Current: 8.23 in, Target: 12.00 in
```

This tells you:
1. **What failed**: Position control
2. **Why it failed**: Couldn't reach target in time
3. **Actual values**: Current vs. expected

### Next Steps After Failure

1. **Check Console Details**: Full error message has specifics
2. **Review AdvantageScope Logs**: See exact sensor values over time
3. **Verify Hardware**: Check connections, motors, sensors
4. **Check Configuration**: Verify constants, PID values, CAN IDs
5. **Test Manually**: Try controlling subsystem via SmartDashboard sliders
6. **Compare to Sim**: Does it work in simulation? Check if sim models realistic behavior

---

## Common Sim-to-Real Issues

### Issue: "Works in sim, position control fails on real robot"
**Possible Causes:**
- PID constants tuned for sim physics, not real mass/friction
- Mechanical binding or resistance not modeled in sim
- Motor current limits too restrictive

**Solution:**
- Retune PID on real robot
- Check for mechanical issues
- Increase current limits if appropriate

---

### Issue: "Sensors return invalid data (NaN/Infinite)"
**Possible Causes:**
- CAN bus errors
- Sensor not connected or powered
- Wrong device ID in code

**Solution:**
- Check CAN bus connections and termination
- Verify sensor power (LED indicator)
- Confirm device IDs match configuration

---

### Issue: "Movement is jerky or oscillates"
**Possible Causes:**
- PID tuning too aggressive
- Sensor noise
- Mechanical backlash

**Solution:**
- Reduce PID gains (especially D term)
- Add sensor filtering
- Check mechanical assembly

---

### Issue: "Motors don't respond"
**Possible Causes:**
- Motor controller not enabled
- CAN ID mismatch
- Motor controller fault
- Inverted incorrectly

**Solution:**
- Check motor controller status LEDs
- Verify CAN IDs in code vs. physical devices
- Clear motor controller faults (power cycle)
- Test motor direction manually first

---

## Advanced Usage

### Creating Custom Tests

To add a test for a new subsystem:

1. **Create a Tester Class**:
```java
public class MySubsystemTester extends SubsystemTester {
  public MySubsystemTester(MySubsystem subsystem) {
    super("MySubsystem");
    // ...
  }

  @Override
  protected void defineTestSteps() {
    testSteps.add(new TestStep("My Test", false) {
      // Implement test logic
    });
  }

  @Override
  protected void returnToSafeState() {
    // Stop subsystem safely
  }
}
```

2. **Register in RobotContainer**:
```java
testManager.registerTester(
    SubsystemTestMode.MY_SUBSYSTEM,
    new MySubsystemTester(mySubsystem)
);
```

### Test Step Structure

Each test step has four phases:
1. **onStart()**: Initialize test (set initial values)
2. **onExecute()**: Run each periodic cycle
3. **isStepComplete()**: Return true when done
4. **validateStep()**: Check if step passed

Mark steps as **critical** if failure should stop the entire test.

---

## Troubleshooting

### Test Won't Start
- Verify robot is enabled
- Check that a test mode is selected in chooser
- Look for errors in DriverStation console

### Test Stops Immediately
- Critical step failed
- Check console for failure reason
- Review hardware connections

### Can't See Test Data
- Verify NetworkTables connection to Dashboard
- Check that `testDashboard.updateDashboard()` is being called
- Open AdvantageScope and load log file

### Test Results Inconsistent
- Mechanical issues causing variability
- Sensor noise
- Battery voltage affecting performance
- Try multiple test runs to identify patterns

---

## Best Practices

1. **Test in Order of Dependency**
   - Test Drive first (most independent)
   - Then individual mechanisms
   - Finally integrated systems

2. **Keep Robot Safe**
   - Always use blocks/stands
   - Start with low power tests
   - Have emergency stop ready

3. **Document Your Findings**
   - Save AdvantageScope logs
   - Note specific error messages
   - Record what changes fixed issues

4. **Iterate Quickly**
   - Make small changes
   - Retest after each change
   - Don't change multiple things at once

5. **Compare to Known-Good**
   - Test on proven working robot if available
   - Compare sensor readings between robots
   - Validate expected ranges

---

## Integration with Competition Code

The testing system is designed to coexist with your competition code:

- **Test Mode Only**: Tests are primarily for TEST mode
- **No Competition Impact**: Tests don't run during matches
- **Disabled by Default**: Normal operation when no test selected
- **Quick Access**: Button combos allow rapid testing during practice

**During Competition:**
- Set test mode to "NONE"
- Testing system is dormant
- Zero performance impact

**During Practice:**
- Quick subsystem validation
- Verify repairs/changes
- Diagnostic troubleshooting

---

## Support and Contribution

This testing framework is part of your team's codebase. As you encounter new issues or develop better tests:

1. Update tester classes with new test steps
2. Add new common issues to this documentation
3. Share improvements with your team
4. Consider contributing back to the FRC community

---

## Summary

The Subsystem Testing System transforms the sim-to-real transition from guesswork into a systematic process:

✅ Isolate subsystems
✅ Automated validation
✅ Clear diagnostics
✅ Rapid troubleshooting
✅ Documented results

Use it every time you:
- Deploy code to a real robot
- Make hardware changes
- Troubleshoot issues
- Validate repairs

**Happy Testing! 🤖**
