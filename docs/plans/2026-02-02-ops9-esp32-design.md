# OPS9 ESP32 Positioning System - Software Architecture Design

**Date:** 2026-02-02
**Target Platform:** ESP32 with PlatformIO + ESP-IDF
**Update Rate:** 200Hz (5ms cycle time)
**Accuracy Goal:** <2cm/m position error, <0.1° heading error

## Overview

This document describes the software architecture for the OPS9 planar positioning system. The system fuses data from a HWT901B IMU and AMT103 encoder to provide accurate 2D position and heading estimates using multi-stage gyroscope error compensation and sensor fusion.

## Design Decisions

- **Framework:** PlatformIO with ESP-IDF (performance + modern tooling)
- **RTOS:** FreeRTOS with multiple tasks (included in ESP-IDF)
- **Data Flow:** Hybrid architecture with shared state and mutexes (minimal latency)
- **Storage:** NVS for calibration data (built-in, wear-leveling)
- **Output:** UART (primary) + WiFi (debugging), ROS2 integration planned
- **Error Handling:** Fail-safe mode with graceful degradation
- **Calibration:** Auto gyro bias on startup, serial commands for full calibration
- **Testing:** Hardware-in-loop testing approach

## System Architecture

### Layer Structure

**1. Hardware Abstraction Layer (HAL)**
- UART driver for HWT901B IMU (200Hz polling with DMA)
- Pulse counter driver for AMT103 encoder (interrupt-driven)
- NVS interface for calibration data persistence
- WiFi driver for debugging output

**2. Sensor Layer**
- IMU task: Reads HWT901B at 200Hz, parses WIT protocol, updates shared state
- Encoder task: Processes pulse interrupts, calculates velocity, updates shared state
- Mutex-protected shared data structures

**3. Processing Layer**
- Main positioning task runs at 200Hz (5ms cycle)
- Algorithm pipeline: IAKF → temp compensation → threshold → attitude solving → coordinate calculation → sensor fusion
- Outputs position (x, y) and heading via UART and WiFi

**4. Calibration & Management Layer**
- Startup calibration (gyro bias, 30s stationary)
- Serial command handler for manual calibration
- NVS read/write for persistent storage
- Fail-safe monitoring and sensor health checks

## FreeRTOS Task Structure

### Task Breakdown

**1. IMU Reader Task**
- Priority: High, Core: 0
- Period: 5ms (200Hz)
- Function: Read UART, parse WIT protocol, update `imu_state_t`
- Stack: 4KB
- Uses hardware UART with DMA

**2. Encoder Handler Task**
- Priority: High, Core: 0
- Trigger: Pulse counter interrupts
- Function: Calculate wheel velocity, update `encoder_state_t`
- Stack: 2KB
- Minimal ISR processing

**3. Position Processing Task**
- Priority: Medium-High, Core: 1
- Period: 5ms (200Hz), synchronized with IMU
- Function: Execute full algorithm pipeline, update `position_state_t`
- Stack: 8KB (Kalman matrices)

**4. UART Output Task**
- Priority: Medium, Core: 1
- Period: 5ms (200Hz) or on-demand
- Function: Format and send position data via UART
- Stack: 2KB
- Non-blocking DMA writes

**5. WiFi Debug Task**
- Priority: Low, Core: 1
- Period: 100ms (10Hz)
- Function: Send diagnostic data over WiFi (UDP/TCP)
- Stack: 4KB

**6. Calibration & Command Task**
- Priority: Low, Core: 1
- Function: Handle serial commands, NVS operations, sensor health monitoring
- Stack: 4KB

### Timing Budget (5ms cycle)

- Sensor read + mutex: ~0.5ms
- IAKF + compensation: ~1.0ms
- Attitude solving (RK2): ~1.5ms
- Coordinate calc + fusion: ~1.0ms
- Output formatting: ~0.5ms
- **Total: ~4.5ms** (0.5ms safety margin)

## Data Structures

### Shared State

```c
// IMU state (updated by IMU task, read by processing task)
typedef struct {
    float omega_x, omega_y, omega_z;  // rad/s
    float accel_x, accel_y, accel_z;  // m/s²
    float temperature;                 // °C
    uint32_t timestamp_us;             // microseconds
    bool valid;                        // data validity flag
    SemaphoreHandle_t mutex;
} imu_state_t;

// Encoder state (updated by encoder task, read by processing task)
typedef struct {
    float velocity;                    // m/s
    uint32_t pulse_count;              // total pulses
    uint32_t timestamp_us;
    bool valid;
    SemaphoreHandle_t mutex;
} encoder_state_t;

// Position state (updated by processing task, read by output tasks)
typedef struct {
    float x, y;                        // meters
    float heading;                     // radians
    float confidence;                  // 0.0-1.0
    uint32_t timestamp_us;
    bool fail_safe_mode;               // true if degraded
    SemaphoreHandle_t mutex;
} position_state_t;

// Calibration data (loaded from NVS at startup)
typedef struct {
    float gyro_bias[3];                // rad/s offset
    float temp_drift_coeffs[3];        // polynomial coefficients
    float wheel_angle;                 // non-orthogonal correction
    float iakf_params[5];              // IAKF tuning parameters
    bool valid;                        // calibration present
} calibration_t;
```

### Memory Allocation

- All shared state: Static allocation (no heap fragmentation)
- Algorithm working memory: Stack-allocated in processing task
- Kalman matrices: Pre-allocated static buffers
- **Total RAM estimate: ~40KB** (ESP32 has 520KB available)

## Algorithm Pipeline

### Processing Task Main Loop (200Hz)

1. **Acquire Sensor Data**
   - Lock IMU mutex, copy imu_state
   - Lock encoder mutex, copy encoder_state
   - Check validity flags

2. **IAKF Stage** (Improved Adaptive Kalman Filter)
   - Input: raw ωz from IMU
   - Apply innovation-based adaptive gain
   - Output: filtered ωz_iakf

3. **Temperature Drift Compensation**
   - Input: ωz_iakf, current temperature
   - Apply polynomial correction: ωz_comp = ωz_iakf - f(T)
   - Use calibration_t.temp_drift_coeffs

4. **Threshold Processing**
   - Input: ωz_comp
   - Apply dead-zone threshold (suppress drift at rest)
   - Output: ωz_threshold

5. **Dynamic State Compensation**
   - Input: ωz_threshold, encoder velocity
   - Detect motion state, apply state-specific corrections
   - Output: ωz_final

6. **Attitude Solving** (Quaternion Integration)
   - Input: ωz_final, ωx, ωy (from IMU)
   - 2nd-order Runge-Kutta integration (dt = 0.005s)
   - Update quaternion q = [q0, q1, q2, q3]
   - Extract heading: θ = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2² + q3²))

7. **Coordinate Calculation**
   - Input: encoder velocity, heading θ
   - Apply non-orthogonal wheel correction (auxiliary coordinate system)
   - Calculate: dx = v * cos(θ) * dt, dy = v * sin(θ) * dt
   - Accumulate: x += dx, y += dy

8. **Sensor Fusion** (Heading Correction)
   - Compare gyro heading vs wheel odometry heading
   - Apply weighted fusion based on motion state
   - Update final heading estimate

9. **Update position_state_t** with results

### Module Organization

- `iakf.c/h` - Kalman filter implementation
- `attitude.c/h` - Quaternion math and RK2 integration
- `coordinate.c/h` - Position calculation and wheel correction
- `fusion.c/h` - Sensor fusion logic
- `compensation.c/h` - Temperature and threshold processing

## Calibration System

### Startup Calibration Sequence

1. **Power-on (t=0s)**
   - Load calibration_t from NVS
   - If no calibration found, use default values, set flag

2. **Gyro Bias Calibration (t=0-30s)**
   - System must remain stationary
   - Sample IMU at 200Hz for 30 seconds
   - Calculate mean of ωx, ωy, ωz
   - Store as gyro_bias[3], save to NVS
   - LED/serial feedback during calibration

3. **Ready State (t=30s+)**
   - Begin normal operation
   - If calibration invalid, output warning flag

### Manual Calibration Commands (via UART)

- `CAL_GYRO` - Run gyro bias calibration (30s stationary)
- `CAL_TEMP_START` - Begin temperature drift calibration
- `CAL_TEMP_SAMPLE` - Record sample at current temperature
- `CAL_TEMP_COMPUTE` - Fit polynomial, save coefficients
- `CAL_WHEEL <angle>` - Set non-orthogonal wheel angle
- `CAL_RESET` - Clear all calibration, use defaults
- `CAL_STATUS` - Display current calibration values

## Fail-Safe System

### Fail-Safe Mode Triggers

**1. IMU Communication Loss**
- No valid data for >50ms (10 cycles)
- Switch to wheel-only odometry
- Heading holds last known value
- Set fail_safe_mode flag, confidence = 0.5

**2. Encoder Signal Loss**
- No pulses detected during expected motion
- Continue with gyro-only positioning
- Position drift increases over time
- Set fail_safe_mode flag, confidence = 0.3

**3. Data Anomaly Detection**
- Angular velocity > 10 rad/s (unrealistic)
- Acceleration > 5g (unrealistic for ground robot)
- Discard bad samples, use previous values
- Log warning, don't enter fail-safe unless persistent

### Recovery

- Automatic recovery when valid data resumes
- Clear fail_safe_mode flag
- Gradual confidence restoration over 1 second

## Output Interfaces

### UART Output (Primary Interface)

**Protocol:** Binary packet format for efficiency at 200Hz

```
Header: 0xAA 0x55 (sync bytes)
Length: 1 byte
Payload:
  - timestamp: uint32_t (microseconds)
  - x: float (meters)
  - y: float (meters)
  - heading: float (radians)
  - confidence: float (0.0-1.0)
  - flags: uint8_t (bit 0: fail_safe_mode, bit 1: calibration_valid)
Checksum: uint8_t (XOR of all bytes)
Total: 23 bytes per packet
```

**Baud rate:** 115200 (sufficient for 200Hz * 23 bytes = 4.6KB/s)

### WiFi Debug Output (Secondary Interface)

- UDP broadcast at 10Hz to port 9090
- JSON format for easy parsing:

```json
{
  "pos": {"x": 1.234, "y": 5.678, "heading": 1.57},
  "sensors": {"imu_valid": true, "enc_valid": true},
  "state": {"fail_safe": false, "confidence": 0.95},
  "debug": {"omega_z": 0.05, "velocity": 0.5, "temp": 25.3}
}
```

- Can be viewed with simple Python script or web dashboard
- Useful for calibration visualization and debugging

### Command Input (UART)

- ASCII commands terminated by newline
- Format: `CMD_NAME [args]\n`
- Responses: `OK\n` or `ERROR: message\n`
- Non-blocking parser in calibration task

### Future ROS2 Integration

- UART interface designed to be compatible with micro-ROS
- Position output maps to `nav_msgs/Odometry`
- Confidence and fail-safe flags map to `diagnostic_msgs/DiagnosticStatus`
- Can add micro-ROS transport layer without changing core algorithm

## Implementation Notes

### Performance Considerations

- Use `-O2` optimization for release builds
- Enable FPU for floating-point operations
- Consider fixed-point math if timing budget exceeded
- Profile critical sections with ESP-IDF profiling tools

### Code Quality

- Use consistent naming conventions (snake_case for C)
- Document all public APIs with Doxygen comments
- Add assertions for critical invariants
- Use static analysis tools (cppcheck, clang-tidy)

### Testing Strategy

- Hardware-in-loop testing on actual robot platform
- Compare against ground truth (motion capture, laser positioning)
- Test edge cases: sensor failures, rapid motion, temperature changes
- Validate calibration procedures with known reference data

## Project Structure

```
ops9/
├── docs/
│   └── plans/
│       └── 2026-02-02-ops9-esp32-design.md
├── reference/
│   ├── datasheets/
│   └── WitStandardProtocol_JY901-main/
├── src/
│   ├── main.c
│   ├── hal/
│   │   ├── imu_driver.c/h
│   │   ├── encoder_driver.c/h
│   │   ├── nvs_manager.c/h
│   │   └── wifi_manager.c/h
│   ├── sensors/
│   │   ├── imu_task.c/h
│   │   └── encoder_task.c/h
│   ├── processing/
│   │   ├── position_task.c/h
│   │   ├── iakf.c/h
│   │   ├── attitude.c/h
│   │   ├── coordinate.c/h
│   │   ├── fusion.c/h
│   │   └── compensation.c/h
│   ├── output/
│   │   ├── uart_output.c/h
│   │   └── wifi_output.c/h
│   ├── calibration/
│   │   ├── calibration_task.c/h
│   │   └── command_parser.c/h
│   └── common/
│       ├── types.h
│       └── config.h
├── test/
│   └── (hardware test scripts)
├── platformio.ini
└── README.md
```

## Next Steps

1. Initialize git repository
2. Set up PlatformIO project with ESP-IDF
3. Implement in stages (see implementation plan)
4. Test each stage before proceeding
5. Commit successful stages to git
