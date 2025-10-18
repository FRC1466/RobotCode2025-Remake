# Subsystem Testing System - Quick Reference

## Quick Start
1. Place robot on blocks
2. Enable robot in TEST mode
3. Select test from "Subsystem Test Mode" chooser
4. Hold **Y + Start** to begin test
5. Hold **B + Start** to stop test

## Test Controls
| Action | Button Combo |
|--------|-------------|
| Start Test | Y + Start |
| Stop Test | B + Start |

## Available Tests
- **NONE**: Normal operation
- **DRIVE**: Gyro, odometry, movement
- **ELEVATOR**: Position control, sensors
- **PIVOT**: Angle control
- **INTAKE**: Rollers, sensors
- **ALL**: Run all tests

## Reading Results

### Console
```
✅ PASS - Test completed successfully
⚠️ WARNING - Issue detected (non-critical)
❌ FAIL - Critical failure
```

### Dashboard
- **Test/CurrentMode**: Active test
- **Test/Progress**: % complete
- **Test/Health**: Pass/Fail/Warning status
- **Test/Summary**: Quick overview

### AdvantageScope
All logs under: `Testing/[Subsystem]/`

## Common Issues

| Problem | Check |
|---------|-------|
| Invalid sensors | CAN connections, device IDs |
| Can't reach position | PID tuning, mechanical binding |
| High current | Friction, damaged motors |
| Position drift | Brake mode, gravity compensation |
| Gyro drift | IMU calibration, connection |

## Safety Checklist
- [ ] Robot on blocks
- [ ] Area clear
- [ ] Emergency stop ready
- [ ] Battery charged
- [ ] CAN termination verified

## Next Steps After Failure
1. Read console error message
2. Check AdvantageScope logs
3. Verify hardware connections
4. Review configuration values
5. Test subsystem manually
6. Compare sim vs. real behavior

## Pro Tips
- Test Drive subsystem first
- One subsystem at a time
- Save AdvantageScope logs
- Document all findings
- Make small changes, retest often

---
For detailed documentation, see `SUBSYSTEM_TESTING.md`
