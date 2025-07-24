# VESC CAN Protocol Documentation

## Hardware Testing Results


## VESC CAN Protocol: Ticks, Pulses, and Odometry

### Key Facts (2025 empirical calibration)
- **3 Hall sensors** = 6 electrical states per mechanical revolution
- **23 poles** (tachometer pulses per revolution, mechanical)
- **138 ticks per mechanical revolution** (23 × 6)
- **Wheel diameter**: 0.3556m
- **Wheel circumference**: π × 0.3556m = 1.117m
- **Distance per tick**: 1.117m / 138 ≈ 0.008094m (8.1 mm per tick)

### CAN Protocol and Odometry
- VESC STATUS_5 messages report the tachometer value as electrical revolutions.
- The odometry node divides by 6 to get mechanical revolutions.
- For each mechanical revolution, the wheel moves 1.117m and the tachometer increases by 138 ticks.
- Use 138 as the correct number of ticks per wheel turn for all odometry and calibration calculations.

### Example Conversion
- If you observe a change of 69 ticks, the wheel has turned 0.5 revolutions (69/138), which is 0.5585m of travel.

### Duty Cycle to Distance
- The mapping from duty cycle to ticks/sec is empirical (see calibration CSV).
- To convert duty cycle to distance/sec: (ticks/sec per duty) × 8.1 mm/tick.

---
Below: original protocol details remain for reference.

### Verified Hardware Configuration
- **Left VESC**: ID 28 (0x1C) → CAN ID `0x0000001C`
- **Right VESC**: ID 46 (0x2E) → CAN ID `0x0000002E`
- **CAN Interface**: can0
- **Command Type**: Duty Cycle (primary control method)

### CAN Message Format

#### Duty Cycle Command Structure
```
CAN ID: 0x0000001C (left) or 0x0000002E (right)
DLC: 4 bytes
Data: [MSB] [   ] [   ] [LSB]
      Byte0 Byte1 Byte2 Byte3
```

#### Data Encoding
- **Format**: 32-bit signed integer, big-endian
- **Range**: -100000 to +100000
- **Scaling**: Represents -100% to +100% duty cycle
- **Zero**: 0x00000000 = 0% duty cycle (motor stop)

### Verified Message Examples

#### Left VESC (ID 28, CAN ID 0x1C)
```bash
# Negative duty cycles (reverse)
cansend can0 0000001C#FF.FF.D8.F0  # -10000 = -10% duty cycle
cansend can0 0000001C#FF.FF.E0.00  # -8192  = -8.192% duty cycle
cansend can0 0000001C#FF.FF.E7.10  # -6384  = -6.384% duty cycle
cansend can0 0000001C#FF.FF.EE.20  # -4576  = -4.576% duty cycle
cansend can0 0000001C#FF.FF.F6.30  # -2512  = -2.512% duty cycle

# Zero and positive duty cycles (forward)
cansend can0 0000001C#00.00.00.00  # 0      = 0% duty cycle (STOP)
cansend can0 0000001C#00.00.07.D0  # 2000   = 2% duty cycle
cansend can0 0000001C#00.00.0F.A0  # 4000   = 4% duty cycle
cansend can0 0000001C#00.00.17.70  # 6000   = 6% duty cycle
cansend can0 0000001C#00.00.1F.40  # 8000   = 8% duty cycle
cansend can0 0000001C#00.00.27.10  # 10000  = 10% duty cycle
```

#### Right VESC (ID 46, CAN ID 0x2E)
```bash
# Negative duty cycles (reverse)
cansend can0 0000002E#FF.FF.D8.F0  # -10000 = -10% duty cycle
cansend can0 0000002E#FF.FF.E0.00  # -8192  = -8.192% duty cycle
cansend can0 0000002E#FF.FF.E7.10  # -6384  = -6.384% duty cycle
cansend can0 0000002E#FF.FF.EE.20  # -4576  = -4.576% duty cycle
cansend can0 0000002E#FF.FF.F6.30  # -2512  = -2.512% duty cycle

# Zero and positive duty cycles (forward)
cansend can0 0000002E#00.00.00.00  # 0      = 0% duty cycle (STOP)
cansend can0 0000002E#00.00.07.D0  # 2000   = 2% duty cycle
cansend can0 0000002E#00.00.0F.A0  # 4000   = 4% duty cycle
cansend can0 0000002E#00.00.17.70  # 6000   = 6% duty cycle
cansend can0 0000002E#00.00.1F.40  # 8000   = 8% duty cycle
cansend can0 0000002E#00.00.27.10  # 10000  = 10% duty cycle
```

### Duty Cycle Scaling Implementation

#### Software to VESC Conversion
```cpp
// Input: duty_cycle (-1.0 to +1.0)
// Output: VESC format (-100000 to +100000)
int32_t vesc_value = static_cast<int32_t>(duty_cycle * 100000);

// Examples:
// duty_cycle = -0.10 → vesc_value = -10000 → 0xFFFFD8F0
// duty_cycle =  0.00 → vesc_value =      0 → 0x00000000
// duty_cycle = +0.10 → vesc_value = +10000 → 0x00002710
```

#### Big-Endian Byte Packing
```cpp
frame.data[0] = (vesc_value >> 24) & 0xFF;  // MSB
frame.data[1] = (vesc_value >> 16) & 0xFF;
frame.data[2] = (vesc_value >> 8) & 0xFF;
frame.data[3] = vesc_value & 0xFF;          // LSB
```

### Safety Considerations

#### Recommended Limits
- **Maximum Duty Cycle**: ±10% for initial testing
- **Deadband**: 1% minimum to overcome motor friction
- **Emergency Stop**: 0% duty cycle (0x00000000)

#### Progressive Testing
1. Start with ±2% duty cycle
2. Gradually increase to ±5%
3. Maximum ±10% for normal operation
4. Higher duty cycles only after thorough testing

### Integration Notes

#### Velocity to Duty Cycle Mapping
```cpp
// Assume max velocity of 2.0 m/s at 100% duty cycle
double duty_cycle = velocity / 2.0;  // Basic linear mapping
duty_cycle = std::clamp(duty_cycle, -1.0, 1.0);
```

#### Differential Drive Implementation
```cpp
// Convert cmd_vel to wheel velocities
double left_velocity = linear - (angular * wheel_separation / 2.0);
double right_velocity = linear + (angular * wheel_separation / 2.0);

// Convert to duty cycles
double left_duty = velocityToDutyCycle(left_velocity);
double right_duty = velocityToDutyCycle(right_velocity);

// Send CAN commands
sendDutyCycleCommand(LEFT_VESC_ID, left_duty);
sendDutyCycleCommand(RIGHT_VESC_ID, right_duty);
```

### Testing Commands

#### Manual Testing
```bash
# Test left wheel forward at 5%
cansend can0 0000001C#00.00.13.88

# Test right wheel forward at 5%  
cansend can0 0000002E#00.00.13.88

# Stop both wheels
cansend can0 0000001C#00.00.00.00
cansend can0 0000002E#00.00.00.00

# Test rotation (left reverse, right forward at 3%)
cansend can0 0000001C#FF.FF.F4.48  # Left reverse
cansend can0 0000002E#00.00.0B.B8  # Right forward
```

#### Monitoring
```bash
# Monitor CAN traffic
candump can0

# Monitor specific VESC responses
candump can0 | grep -E "(1C|2E)"
```

This protocol documentation is based on verified hardware testing and should be used as the reference for implementation.
