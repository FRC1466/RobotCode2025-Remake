# Running Subsystem Tests in Simulation

## ✅ Yes, It Works in Sim!

The testing system **fully supports simulation** and is actually a great way to:
- Verify the testing framework works
- Develop and debug test sequences
- Test the test system before deploying to real robot
- Practice using the testing workflow

---

## 🎮 How to Run Tests in Sim

### Method 1: Using WPILib Sim GUI

1. **Start the simulator**
   ```powershell
   # In VS Code, press F5 or run:
   ./gradlew simulateJava
   ```

2. **Enable Robot in TEST mode**
   - In the sim GUI, select "Test" mode
   - Click "Enable"

3. **Open Shuffleboard/Glass**
   - Launch Shuffleboard or Glass
   - Connect to localhost (should auto-connect)

4. **Select Test Mode**
   - Find "Subsystem Test Mode" chooser
   - Select subsystem to test (e.g., "Elevator")

5. **Start Test**
   - Hold **Y + Start** on your controller
   - OR create a button in Shuffleboard to trigger the test command

6. **Watch Results**
   - Console in VS Code shows step-by-step results
   - Shuffleboard shows test progress and health
   - AdvantageScope can replay logs after

---

## 📊 What Works in Sim

### ✅ **Fully Functional**

| Subsystem | What's Tested | Sim Support |
|-----------|--------------|-------------|
| **Elevator** | Position control, sensor readings | ✅ Full (ElevatorIOSim) |
| **Pivot** | Angle control, sensor readings | ✅ Full (PivotIOSim) |
| **Intake** | Motor control, basic operation | ✅ Full (RollerSystemIOSim) |
| **Drive** | Odometry, gyro, basic movement | ✅ Full (DriveIOSim/CTRE) |

### ⚠️ **Partial Support**

| Feature | Limitation | Workaround |
|---------|-----------|------------|
| Game Piece Sensors | No physical sensors in sim | Manually set `intake.setHasCoral(true)` |
| Current Monitoring | Simulated values may not match real | Expect different readings |
| Physical Movement | No actual motors moving | Watch simulated position values |

### ❌ **Won't Work**

- Physical motor temperature readings (will be simulated/default values)
- Beam break sensors (unless manually triggered)
- Color sensors for coral detection
- Actual mechanical issues (friction, binding, etc.)

---

## 🎯 What You'll See in Sim

### Console Output (Same as Real Robot)
```
⚠️ CHOREOGRAPHER DISABLED FOR TESTING
=== Starting Elevator Test ===
Checking elevator sensors...
✓ Sensors connected. Position: 0.00 in, Velocity: 0.00 in/s
✓ Motor current monitoring available
Moving to mid position: 12.0 inches
✓ Reached mid position in 0.87 seconds
Checking position stability...
✓ Position stable, drift: 0.000 inches
Returning to home position...
✓ Returned to home position
=== Elevator Test Complete ===
Result: PASSED

Elevator Health: ✅ Pass
  ✅ Sensor Connectivity Check: Step completed successfully
  ✅ Motor Current Check: Step completed successfully
  ✅ Position Control Test (Mid): Step completed successfully
  ✅ Position Stability: Step completed successfully
  ✅ Return to Home: Step completed successfully

✅ CHOREOGRAPHER RE-ENABLED
```

### Shuffleboard Display
- Test/CurrentMode: "Elevator"
- Test/Active: true
- Test/Progress: 100%
- Test/Health: "✅ Pass"
- Test/Passed: 5
- Test/Failed: 0

---

## 🔧 Sim-Specific Considerations

### 1. **Physics Simulation**
Simulated subsystems may behave differently:
- **Faster movement** - No real mass/inertia
- **Perfect control** - No mechanical slop
- **Instant response** - No motor lag

**What to do:**
- Tests may complete faster in sim
- Use sim to verify test logic, not physics accuracy
- Real robot testing still required for actual validation

### 2. **Sensor Simulation**
Some sensors are simulated:
```java
// In sim, sensors return idealized values
elevator.getPosition()  // Perfect position tracking
pivot.getAngle()        // Perfect angle tracking
drive.getPose()         // Perfect odometry
```

**What to do:**
- Verify sensor connectivity checks pass
- Don't expect noise or drift in sim
- Real robot will have more variation

### 3. **Manual Triggers Needed**
Some things require manual input:
```java
// Game piece detection
intake.setHasCoral(true);  // Manually set in code or dashboard
intake.setHasAlgae(true);  // for sensor tests
```

**What to do:**
- Add dashboard toggles for manual sensor triggers
- Or modify test to skip sensor validation in sim
- Or use sim-specific test variants

---

## 💡 Best Practices for Sim Testing

### 1. **Start with Sim**
- Develop tests in simulation first
- Debug test logic without hardware risk
- Verify test sequence makes sense
- Check console output formatting

### 2. **Verify Test Framework**
Run through all test modes in sim:
```
✅ Test Mode Chooser works
✅ Y + Start triggers tests
✅ B + Start stops tests
✅ Console output appears
✅ Dashboard updates
✅ Choreographer disables/enables
✅ Health monitoring works
```

### 3. **Compare Sim vs Real**
After running in sim, note differences when running on real robot:
- Timing differences
- Sensor noise
- Mechanical issues
- Current draw patterns

### 4. **Use for Development**
Sim is perfect for:
- Adding new test steps
- Testing failure scenarios
- Debugging test logic
- Training new team members

---

## 🚀 Quick Sim Test Workflow

### Option 1: Full Sim Test
```powershell
# 1. Start sim
./gradlew simulateJava

# 2. In Sim GUI
#    - Mode: Test
#    - Click Enable

# 3. In VS Code terminal, you should see:
#    Robot code starting...

# 4. Use controller to trigger test
#    Hold Y + Start

# 5. Watch console output
```

### Option 2: AdvantageScope Replay
```powershell
# 1. Run sim, perform tests
# 2. Sim auto-logs to AdvantageKit
# 3. Open AdvantageScope
# 4. Load log file from logs folder
# 5. Review Testing/ data tree
```

---

## 📝 Example: Testing Elevator in Sim

### Step-by-Step
1. **Start Simulation**
   ```powershell
   ./gradlew simulateJava
   ```

2. **Enable in Test Mode**
   - Sim GUI → Test → Enable

3. **Open Console**
   - Watch VS Code terminal

4. **Select Test**
   - Shuffleboard → "Subsystem Test Mode" → "Elevator"

5. **Start Test**
   - Controller: Hold Y + Start
   - OR Shuffleboard: Create button for `testManager.startSelectedTest()`

6. **Observe**
   ```
   ⚠️ CHOREOGRAPHER DISABLED FOR TESTING
   === Starting Elevator Test ===
   [Watch test progress]
   === Elevator Test Complete ===
   Result: PASSED
   ✅ CHOREOGRAPHER RE-ENABLED
   ```

7. **Review Results**
   - Check console for details
   - Verify all steps passed
   - Note any warnings

---

## ⚙️ Sim Configuration

Your current setup:
```java
// In Constants.java
private static RobotType robotType = RobotType.SIMBOT;

// In RobotContainer.java (when SIMBOT)
elevator = new Elevator(new ElevatorIOSim(), new HomeSensorIO() {});
wrist = new Pivot(new PivotIOSim());
intake = new Intake(
    new RollerSystemIOSim(DCMotor.getKrakenX60(1), 1, 1),
    new RollerSystemIOSim(DCMotor.getNeoVortex(1), 1, 1),
    new CoralSensorIO() {});
```

**All sim IO classes are already implemented!** ✅

---

## 🎓 Learning Value

Running tests in sim helps you:

### 1. **Learn the System**
- Understand test workflow
- See what each test does
- Learn button controls
- Practice interpreting results

### 2. **Debug Tests**
- Find logic errors in test steps
- Fix timing issues
- Improve test messages
- Validate test sequences

### 3. **Train Team**
- Show new members how testing works
- Practice without robot access
- Demonstrate test results
- Build confidence before real testing

### 4. **Develop New Tests**
- Prototype new test steps in sim
- Verify test logic
- Test edge cases
- Iterate quickly

---

## ⚠️ Important Limitations

### What Sim CANNOT Validate

1. **Real Hardware Issues**
   - CAN bus errors
   - Actual motor problems
   - Physical sensor failures
   - Wiring issues
   - Mechanical binding

2. **Real Physics**
   - Actual mass and inertia
   - Real friction
   - Battery voltage effects
   - Current draw patterns

3. **Timing Precision**
   - Sim runs at variable speed
   - Not real-time accurate
   - Loop timing different

### What You MUST Test on Real Robot

✅ Actual hardware connectivity
✅ Real sensor readings
✅ Mechanical movement
✅ Current draw validation
✅ PID tuning accuracy
✅ Performance under load

---

## 🎯 Summary

### ✅ **Sim Testing is Great For:**
- Learning the testing system
- Developing test sequences
- Debugging test logic
- Training and demonstration
- Quick iteration

### ❌ **Sim Testing is NOT Enough For:**
- Validating real hardware
- Finding mechanical issues
- Measuring actual performance
- Competition readiness

### 🎯 **Best Practice:**
1. **Develop tests in sim** - Fast iteration, safe environment
2. **Verify framework works** - All features functional
3. **Deploy to real robot** - Actual validation
4. **Use both together** - Sim for development, real for validation

---

## 🚀 Try It Now!

```powershell
# Run this to start testing in sim:
./gradlew simulateJava

# Then:
# 1. Enable in Test mode
# 2. Hold Y + Start on controller
# 3. Watch the magic happen!
```

**Yes, it absolutely works in sim, and you should try it!** It's a great way to verify the system before putting it on your robot. 🎉

---

## Next Steps

After sim testing:
1. ✅ Verify all test modes work
2. ✅ Practice using controls
3. ✅ Review log output
4. ✅ Deploy to real robot
5. ✅ Compare sim vs real results
6. ✅ Identify real-world issues

**Have fun testing! 🤖**
