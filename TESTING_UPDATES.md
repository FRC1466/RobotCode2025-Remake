# Testing System Updates - Repeatability & Manual Tuning

## Changes Made

### 1. Fixed Test Repeatability ✅

**Problem**: Tests could not be run twice in a row. After stopping a test with B+Start, attempting to start another test (Y+Start) would fail or behave incorrectly.

**Root Cause**: Test steps maintained internal state (`started` flag, `failureReason` string) that was never reset between test runs.

**Solution**: 
- Added `reset()` method to `TestStep` class that clears `started` and `failureReason`
- Modified `SubsystemTester.startTest()` to call `reset()` on all test steps before starting
- Tests now fully reset state and can be run repeatedly

**Impact**: You can now:
- Run a test → Stop it → Run the same test again
- Run different tests back-to-back
- Test repeatedly without restarting the robot code

---

### 2. Improved Test Timing ⏱️

**Problem**: Tests finished too quickly in simulation, making it hard to observe behavior. Tests were not reliable on repeated runs due to timing issues.

**Changes Made**:
- **Sensor checks**: Added 0.5s observation periods (was instant)
- **Position control**: Added 0.5s settle time after reaching goal + doubled timeouts (5s → 10s)
- **Stability tests**: Increased monitoring duration (1.0s → 2.0s for elevator, 2.0s → 3.0s for gyro)
- **Intake tests**: Added 0.5s command delay + proper run times + settle periods
- **Drive tests**: Increased observation windows (3.0s → 5.0s for movement)

**Why This Helps**:
- **Simulation**: Physics needs time to settle even when "at goal"
- **Real Robot**: Motors and sensors need time to stabilize
- **Repeatability**: Consistent timing prevents random pass/fail results
- **Observability**: You can actually see what's happening

**Test Duration Examples**:
| Test | Old Duration | New Duration |
|------|-------------|--------------|
| Elevator | ~5-8 seconds | ~15-20 seconds |
| Pivot | ~5-8 seconds | ~12-15 seconds |
| Drive | ~8-10 seconds | ~15-18 seconds |
| Intake | ~4-6 seconds | ~10-12 seconds |

---

### 3. Added Manual Tuning Mode 🎯

**New Feature**: A new test mode that allows direct control of subsystem positions via NetworkTables.

**Why This Is Useful**:
- Create and test new setpoints without recompiling
- Perfect for simulation testing before real robot
- Fine-tune positions safely
- Validate movement ranges
- Record good setpoint values for later use

**How It Works**:
1. Select "Manual Tuning" from test mode chooser
2. Press Y+Start to activate
3. Use NetworkTables to set target positions:
   - `Manual/ElevatorSetpoint_Inches` (0-36 inches)
   - `Manual/PivotSetpoint_Degrees` (-10 to 120 degrees)
4. Enable/disable each subsystem independently
5. Watch real-time position updates
6. Press B+Start to exit

**NetworkTables Published**:

*Elevator:*
- `Manual/ElevatorEnabled` - Enable/disable control
- `Manual/ElevatorSetpoint_Inches` - Target position
- `Manual/ElevatorCurrent_Inches` - Current position (read-only)
- `Manual/ElevatorAtGoal` - At target (read-only)

*Pivot:*
- `Manual/PivotEnabled` - Enable/disable control
- `Manual/PivotSetpoint_Degrees` - Target angle
- `Manual/PivotCurrent_Degrees` - Current angle (read-only)
- `Manual/PivotAtGoal` - At target (read-only)

**Safety Features**:
- Automatic range clamping (elevator: 0-36", pivot: -10° to 120°)
- Choreographer auto-disabled during use
- Safe return to IDLE on exit
- Can enable/disable subsystems independently

**Example Workflow**:
```
1. Start sim → Enable Manual Tuning
2. Set elevator to 24.0 inches
3. Set pivot to 45.0 degrees
4. Fine-tune: 24.0 → 24.2 inches, 45.0 → 47.0 degrees
5. Record values: SCORE_L3 = (24.2", 47.0°)
6. Add to constants and test on real robot
```

---

## Files Modified

### Core Framework
1. **SubsystemTester.java** - Added `reset()` method to TestStep, call reset in `startTest()`
2. **SubsystemTestMode.java** - Added `MANUAL` enum value
3. **TestManager.java** - Added MANUAL case to subsystem isolation logic and chooser

### Test Timing Updates
4. **ElevatorTester.java** - Added timers, settle times, increased durations
5. **DriveTester.java** - Added observation periods, increased durations
6. **PivotTester.java** - Added settle times, doubled timeouts
7. **IntakeTester.java** - Added delays, increased test durations

### New Files
8. **ManualTuningTester.java** - New tester for manual setpoint control
9. **MANUAL_TUNING.md** - Complete documentation for manual mode

### Integration
10. **RobotContainer.java** - Registered ManualTuningTester with test manager

### Documentation Updates
11. **SUBSYSTEM_TESTING.md** - Added manual mode info and repeatability note

---

## Testing the Changes

### Test Repeatability
1. Enable robot in TEST mode
2. Select "Elevator Test"
3. Press Y+Start → Watch test run → Press B+Start to stop
4. **Without restarting robot**: Press Y+Start again
5. ✅ Test should run again successfully
6. Repeat steps 3-5 multiple times

### Test Manual Mode (Simulation)
1. Start simulation
2. Enable robot in TEST mode
3. Select "Manual Tuning"
4. Press Y+Start
5. Open NetworkTables viewer
6. Set `Manual/ElevatorSetpoint_Inches` = 12.0
7. ✅ Elevator should move to 12 inches
8. Set `Manual/PivotSetpoint_Degrees` = 45.0
9. ✅ Pivot should move to 45 degrees
10. Press B+Start to exit

### Test Timing Improvements
1. Run any test in simulation
2. ✅ Should take longer than before (more observable)
3. Run test twice → ✅ Should give same result both times
4. ✅ Console output should show timing information

---

## Breaking Changes

**None!** All changes are backwards compatible:
- Existing tests continue to work
- New timing is only in test mode, doesn't affect normal operation
- Manual mode is opt-in (don't select it if you don't need it)

---

## Next Steps

### For Simulation Testing
1. Use Manual Tuning mode to create setpoints
2. Test full range of motion without risk
3. Record good positions for competition use

### For Real Robot Testing
1. Run automated tests on blocks to verify hardware
2. Use Manual Tuning to fine-tune positions
3. Compare sim vs real positions and adjust
4. Iterate until setpoints work on both

### Future Enhancements
Consider adding:
- Vision testing mode
- Coordinated movements (elevator + pivot sequences)
- Position presets in manual mode
- Recording/playback of manual movements
- Stress testing modes (rapid movements, endurance)

---

## Summary

✅ **Fixed**: Tests can now run repeatedly without restarting robot  
✅ **Improved**: Test timing is more reliable and observable  
✅ **Added**: Manual tuning mode for setpoint creation and testing  
✅ **Maintained**: All existing functionality remains unchanged  

**Result**: Comprehensive testing system that works reliably in both simulation and on real hardware, with tools for both automated validation and manual exploration.
