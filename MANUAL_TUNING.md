# Manual Setpoint Tuning Mode

## Overview
The **Manual Tuning** mode allows you to control subsystem positions directly through NetworkTables. This is perfect for:
- 🎯 **Creating new setpoints** in simulation before testing on real robot
- 🔧 **Fine-tuning positions** without recompiling code
- 📊 **Testing position limits** safely
- 🧪 **Validating movement ranges** in both sim and real life

## How to Use

### 1. Activate Manual Mode
1. Open your dashboard (Shuffleboard, Glass, or Elastic)
2. In the test mode chooser, select **"Manual Tuning"**
3. Press **Y + Start** to begin

### 2. Control Subsystems via NetworkTables

The following NetworkTables entries will be published:

#### Elevator Control
| Key | Type | Description |
|-----|------|-------------|
| `Manual/ElevatorEnabled` | Boolean | Enable/disable elevator control |
| `Manual/ElevatorSetpoint_Inches` | Number | Target position (0-36 inches) |
| `Manual/ElevatorCurrent_Inches` | Number | Current position (read-only) |
| `Manual/ElevatorAtGoal` | Boolean | Whether at target (read-only) |

#### Pivot Control
| Key | Type | Description |
|-----|------|-------------|
| `Manual/PivotEnabled` | Boolean | Enable/disable pivot control |
| `Manual/PivotSetpoint_Degrees` | Number | Target angle (-10 to 120 degrees) |
| `Manual/PivotCurrent_Degrees` | Number | Current angle (read-only) |
| `Manual/PivotAtGoal` | Boolean | Whether at target (read-only) |

#### Instructions
| Key | Type | Description |
|-----|------|-------------|
| `Manual/Instructions` | String | Usage instructions |

### 3. Adjust Positions
- Change the `SetpointInches` or `SetpointDegrees` values
- Subsystems will move to new positions automatically
- Watch the `Current` values update in real-time
- Check `AtGoal` to see when movement completes

### 4. Record Good Setpoints
When you find a good position:
1. Note the current setpoint values
2. Add them as constants in your subsystem code
3. Create named positions for easy use

### 5. Exit Manual Mode
Press **B + Start** to stop and return subsystems to IDLE

## Safety Features

### Automatic Clamping
- **Elevator**: Limited to 0-36 inches (configurable)
- **Pivot**: Limited to -10° to 120° (configurable)
- Values outside these ranges are automatically clamped

### Choreographer Disabled
- The choreographer is automatically disabled during manual mode
- This prevents conflicts between manual control and automated superstructure

### Safe Shutdown
- When exiting, subsystems return to IDLE state
- No abrupt stops or uncontrolled movements

## Example Workflow

### Creating a New Scoring Position (Simulation)

1. **Start Manual Mode**
   ```
   Y + Start → Select "Manual Tuning"
   ```

2. **Move Elevator First**
   ```
   Set: Manual/ElevatorSetpoint_Inches = 24.0
   Wait: Manual/ElevatorAtGoal = true
   ```

3. **Adjust Pivot Angle**
   ```
   Set: Manual/PivotSetpoint_Degrees = 45.0
   Wait: Manual/PivotAtGoal = true
   ```

4. **Fine-Tune Position**
   ```
   Adjust elevator: 24.0 → 23.5 → 24.2 (perfect!)
   Adjust pivot: 45.0 → 46.5 → 47.0 (looks good!)
   ```

5. **Record Values**
   ```java
   // Add to your Constants file:
   public static final double SCORE_L3_ELEVATOR_INCHES = 24.2;
   public static final double SCORE_L3_PIVOT_DEGREES = 47.0;
   ```

6. **Test on Real Robot**
   - Deploy code with new constants
   - Use manual mode to verify position on real hardware
   - Adjust if needed

### Testing Position Limits

1. **Enable one subsystem at a time**
   ```
   Set: Manual/PivotEnabled = false  (disable pivot)
   Keep: Manual/ElevatorEnabled = true
   ```

2. **Test extreme positions safely**
   ```
   Start at: 0 inches
   Increment: 0 → 5 → 10 → 15 → 20 → 25 → 30 → 35
   Observe: Any binding, noise, or issues?
   ```

3. **Document safe ranges**
   ```
   Safe min: 0.5 inches (slight clearance)
   Safe max: 34.0 inches (before hard stop)
   ```

## Dashboard Layouts

### Shuffleboard
Create a "Manual Tuning" tab with:
- Number sliders for setpoints (with min/max limits)
- Boolean boxes for enable toggles
- Text display for current positions
- Boolean indicators for AtGoal status

### Glass
Add a "Manual Tuning" window with:
- NetworkTables view of all `Manual/*` keys
- Numeric controls for setpoints
- Graphs for position over time

### Elastic
Use the testing dashboard layout which includes:
- Quick access to all manual control fields
- Visual feedback for current vs target
- Enable/disable toggles

## Tips & Tricks

### 🎯 Sim Testing
- Test full range of motion without risk
- Iterate quickly on new positions
- Record setpoints before real robot testing

### 🔧 Real Robot
- Start with small movements
- Always enable one subsystem at a time initially
- Watch for mechanical interference
- Listen for unusual sounds

### 📊 Data Collection
- Use AdvantageScope to record position data
- Plot elevator vs pivot positions
- Identify optimal movement sequences
- Create smooth interpolation paths

### ⚡ Quick Tuning
- Use keyboard shortcuts in your dashboard
- Create preset buttons for common positions
- Use step increments (0.5 in, 5°) for quick scanning

## Troubleshooting

### Subsystem Not Moving
- **Check**: Is the subsystem enabled? (`Manual/[Subsystem]Enabled = true`)
- **Check**: Is manual mode active? (Test Active should be true)
- **Check**: Is the setpoint within safe limits?
- **Check**: Are there any error messages in the console?

### Choppy Movement
- **Try**: Smaller setpoint increments
- **Try**: Waiting for AtGoal before next move
- **Check**: Is simulation running at full speed?

### Can't Re-Enter Manual Mode
- **Press**: B + Start to fully exit
- **Wait**: Until "Test Active" shows false
- **Then**: Select manual mode and press Y + Start again

### Positions Don't Match Real Robot
- **Note**: Simulation physics may differ slightly
- **Solution**: Use manual mode on real robot to verify
- **Adjust**: Constants based on real hardware testing

## Integration with Testing Framework

Manual mode integrates seamlessly with the testing framework:
- Uses same test isolation system
- Shares subsystem health monitoring
- Compatible with all dashboard layouts
- Can be interrupted with B + Start like other tests

## Console Output

While in manual mode, you'll see periodic updates:
```
=== Manual Tuning Mode Active ===
Use NetworkTables to control subsystem positions:
  - Manual/ElevatorSetpoint_Inches
  - Manual/PivotSetpoint_Degrees
  - Manual/ElevatorEnabled / Manual/PivotEnabled
Press B+Start to exit.

Elevator: 24.20 in (target: 24.20 in, at goal: true) | Pivot: 47.0° (target: 47.0°, at goal: true)
Elevator: 24.18 in (target: 24.20 in, at goal: false) | Pivot: 47.1° (target: 47.0°, at goal: false)
...

=== Manual Tuning Mode Stopped ===
Subsystems returned to IDLE state.
```

## Related Documentation
- [SUBSYSTEM_TESTING.md](SUBSYSTEM_TESTING.md) - Main testing guide
- [TESTING_QUICKREF.md](TESTING_QUICKREF.md) - Quick reference
- [TESTING_IN_SIM.md](TESTING_IN_SIM.md) - Simulation testing
