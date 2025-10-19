# ⚠️ IMPORTANT: Choreographer Auto-Disable Feature

## What Changed

**The testing system now automatically disables the Choreographer when tests run!**

This prevents the superstructure from overriding test commands and ensures true subsystem isolation.

---

## How It Works

### When You Start a Test (Y + Start):
```
⚠️ CHOREOGRAPHER DISABLED FOR TESTING
=== Starting [Subsystem] Test ===
```

**What happens:**
1. TestManager calls `choreographer.setDisabled(true)`
2. Choreographer's `periodic()` method returns early
3. Subsystems can now be controlled directly by tests
4. No interference from superstructure logic

### When You Stop a Test (B + Start or Disable):
```
=== Test Complete ===
✅ CHOREOGRAPHER RE-ENABLED
```

**What happens:**
1. TestManager calls `choreographer.setDisabled(false)`
2. Choreographer resumes normal operation
3. Robot returns to normal competition behavior

---

## Technical Implementation

### Choreographer.java
```java
@Getter private boolean disabled = false;

@Override
public void periodic() {
  // Skip if disabled by testing system
  if (disabled) {
    Logger.recordOutput("Choreographer/Disabled", true);
    return;
  }

  // Normal choreographer logic...
}

public void setDisabled(boolean disabled) {
  this.disabled = disabled;
  // Prints console messages
}
```

### TestManager.java
```java
private Consumer<Boolean> choreographerDisableCallback = null;

public void setChoreographerDisableCallback(Consumer<Boolean> callback) {
  this.choreographerDisableCallback = callback;
}

public Command startTest(SubsystemTestMode mode) {
  return Commands.runOnce(() -> {
    // Disable choreographer
    if (choreographerDisableCallback != null) {
      choreographerDisableCallback.accept(true);
    }
    // Start test...
  });
}

public void stopTest() {
  // Re-enable choreographer
  if (choreographerDisableCallback != null) {
    choreographerDisableCallback.accept(false);
  }
  // Stop test...
}
```

### RobotContainer.java
```java
// Connect the callback
testManager.setChoreographerDisableCallback(choreographer::setDisabled);
```

---

## Why This Matters

### Without Auto-Disable:
❌ Choreographer would fight with test commands
❌ Elevator test tries to move to 12" → Choreographer says "No, go to stow!"
❌ Test results would be invalid
❌ Hard to tell if subsystem works

### With Auto-Disable:
✅ Test has full control of subsystem
✅ Elevator moves exactly where test commands
✅ Accurate validation of functionality
✅ Clear pass/fail results

---

## What You See

### Console Output:
```
⚠️ CHOREOGRAPHER DISABLED FOR TESTING
⚠️ CHOREOGRAPHER DISABLED - Subsystems can be controlled directly
=== Starting Elevator Test ===
Checking elevator sensors...
✓ Sensors connected. Position: 0.50 in, Velocity: 0.00 in/s
...
=== Elevator Test Complete ===
Result: PASSED
✅ CHOREOGRAPHER RE-ENABLED
✅ CHOREOGRAPHER ENABLED - Normal operation resumed
```

### AdvantageKit Logs:
- `Choreographer/Disabled: true` (during test)
- `Choreographer/Disabled: false` (after test)

---

## Safety Features

1. **Automatic Re-enable**: If test crashes, next test or disable will re-enable
2. **Clear Status**: Console messages confirm state changes
3. **Logged**: AdvantageKit records all enable/disable events
4. **Manual Override**: Can manually stop test with B + Start

---

## Testing Modes Affected

This affects **ALL** test modes:
- ✅ DRIVE test - Choreographer won't interfere
- ✅ ELEVATOR test - Full position control
- ✅ PIVOT test - Direct angle commands
- ✅ INTAKE test - Independent roller control
- ✅ ALL test - Choreographer disabled for entire sequence

---

## Developer Notes

### Adding Similar Disable for Other Systems

If you have other systems that might interfere (not currently in code):

```java
// In TestManager
private Consumer<Boolean> systemDisableCallback = null;

public void setSystemDisableCallback(Consumer<Boolean> callback) {
  this.systemDisableCallback = callback;
}

// Use in startTest() and stopTest()
if (systemDisableCallback != null) {
  systemDisableCallback.accept(true/false);
}

// In RobotContainer
testManager.setSystemDisableCallback(system::setDisabled);
```

### Checking if Choreographer is Disabled

```java
// From anywhere with access to choreographer
if (choreographer.isDisabled()) {
  // Choreographer is disabled, subsystems have direct control
}
```

---

## Comparison: Before vs After

### Before (Without Auto-Disable):
```java
// Test tries to move elevator
elevator.setWantedState(MOVE_TO_POSITION, 12.0);

// BUT... Choreographer also runs
choreographer.periodic() {
  elevator.setWantedState(IDLE);  // Overrides test!
  wrist.setWantedState(STOWED);
  intake.setWantedState(OFF);
}

// Result: Test fails, subsystem never moves
```

### After (With Auto-Disable):
```java
// Test tries to move elevator
elevator.setWantedState(MOVE_TO_POSITION, 12.0);

// Choreographer is disabled
choreographer.periodic() {
  if (disabled) {
    return;  // Early exit, no override!
  }
}

// Result: Test succeeds, subsystem moves as commanded
```

---

## Summary

✅ **Automatic** - No manual intervention needed
✅ **Safe** - Always re-enables after test
✅ **Clear** - Console messages show state
✅ **Logged** - AdvantageKit records everything
✅ **Essential** - Required for accurate testing

**This feature ensures your subsystem tests actually test the subsystems, not the fight between test and choreographer!**

---

## Status

✅ Implemented in Choreographer
✅ Integrated in TestManager
✅ Connected in RobotContainer
✅ Documented in all guides
✅ Ready to use

**The testing system is now fully isolated and won't be interfered with by the superstructure!**
