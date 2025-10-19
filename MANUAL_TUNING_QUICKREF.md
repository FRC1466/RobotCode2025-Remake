# Manual Tuning Quick Reference

## Quick Start
```
1. Select "Manual Tuning" from test chooser
2. Press Y + Start
3. Open NetworkTables viewer
4. Adjust setpoints in real-time
5. Press B + Start to exit
```

## NetworkTables Keys

### Elevator
| Key | Type | Range | R/W |
|-----|------|-------|-----|
| `Manual/ElevatorEnabled` | Boolean | - | Write |
| `Manual/ElevatorSetpoint_Inches` | Number | 0-36 | Write |
| `Manual/ElevatorCurrent_Inches` | Number | - | Read |
| `Manual/ElevatorAtGoal` | Boolean | - | Read |

### Pivot
| Key | Type | Range | R/W |
|-----|------|-------|-----|
| `Manual/PivotEnabled` | Boolean | - | Write |
| `Manual/PivotSetpoint_Degrees` | Number | -10 to 120 | Write |
| `Manual/PivotCurrent_Degrees` | Number | - | Read |
| `Manual/PivotAtGoal` | Boolean | - | Read |

### Info
| Key | Type | R/W |
|-----|------|-----|
| `Manual/Instructions` | String | Read |

## Safety Limits
- **Elevator**: Automatically clamped to 0-36 inches
- **Pivot**: Automatically clamped to -10° to 120°
- Values outside range are adjusted automatically
- Choreographer disabled during manual mode

## Common Workflows

### Create New Setpoint
```
1. Start manual mode
2. Move elevator: Manual/ElevatorSetpoint_Inches = 24.0
3. Wait for: Manual/ElevatorAtGoal = true
4. Move pivot: Manual/PivotSetpoint_Degrees = 45.0
5. Wait for: Manual/PivotAtGoal = true
6. Fine-tune as needed
7. Record final values
```

### Test Position Range
```
1. Disable pivot: Manual/PivotEnabled = false
2. Test elevator: 0 → 5 → 10 → 15 → ... → 35
3. Note any issues
4. Enable pivot: Manual/PivotEnabled = true
5. Disable elevator: Manual/ElevatorEnabled = false
6. Test pivot: 0 → 15 → 30 → 45 → ... → 115
7. Note any issues
```

### Quick Presets (Create in Dashboard)
```
Stowed:     Elevator = 0.0,  Pivot = 0.0
Low Rung:   Elevator = 8.0,  Pivot = 30.0
Mid Rung:   Elevator = 16.0, Pivot = 45.0
High Rung:  Elevator = 24.0, Pivot = 60.0
Max Reach:  Elevator = 32.0, Pivot = 90.0
```

## Console Output Example
```
=== Manual Tuning Mode Active ===
Use NetworkTables to control subsystem positions:
  - Manual/ElevatorSetpoint_Inches
  - Manual/PivotSetpoint_Degrees
Press B+Start to exit.

Elevator: 24.20 in (target: 24.20 in, at goal: true) | Pivot: 47.0° (target: 47.0°, at goal: true)
Elevator: 12.05 in (target: 12.00 in, at goal: false) | Pivot: 30.2° (target: 30.0°, at goal: true)

=== Manual Tuning Mode Stopped ===
Subsystems returned to IDLE state.
```

## Tips
- 🎯 Use in sim first, then verify on real robot
- 🔧 Test one subsystem at a time initially
- 📊 Record all good setpoints immediately
- ⚡ Small increments (0.5", 5°) for safety
- 🎮 Create dashboard presets for common positions

## Troubleshooting
| Issue | Solution |
|-------|----------|
| Subsystem not moving | Check Enabled = true |
| Test won't start | Press B+Start first to exit previous test |
| Values snap back | Outside safe range, will clamp |
| Can't re-enter | Fully exit first (B+Start) |

## See Also
- [MANUAL_TUNING.md](MANUAL_TUNING.md) - Full documentation
- [SUBSYSTEM_TESTING.md](SUBSYSTEM_TESTING.md) - Main testing guide
- [TESTING_IN_SIM.md](TESTING_IN_SIM.md) - Simulation testing
