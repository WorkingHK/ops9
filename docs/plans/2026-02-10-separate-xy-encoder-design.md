# Separate X/Y Encoder Design for OPS9

**Date:** 2026-02-10
**Author:** Design collaboration with user
**Status:** Design complete, ready for implementation

## Overview

This document describes the mathematical model and implementation approach for using **separate X and Y encoders** instead of a combined encoder for the OPS9 positioning system.

## Hardware Configuration

### Encoder Positions (from chassis center at origin)

- **Left encoder (Y-encoder):** Position `(-86.06, -70)` mm
  - Horizontal omniwheel orientation
  - Measures Y-axis motion

- **Right encoder (X-encoder):** Position `(85.94, -70)` mm
  - Vertical omniwheel orientation
  - Measures X-axis motion

### Distance from Rotation Center

- Left wheel: `r_L = 110.841 mm`
- Right wheel: `r_R = 110.934 mm`

## Mathematical Model

### Rigid Body Motion

When the chassis has velocity `(V_x, V_y)` at the rotation center and angular velocity `ω`, any point on the chassis at position `(x, y)` has velocity:

```
V_point = V_center + ω × [x, y]
        = [V_x, V_y] + [-ω×y, ω×x]
```

### Encoder Measurement Model

**Left encoder (measures Y-velocity):**
```
V_y_measured = V_y + ω × x_left
             = V_y + ω × (-86.06)
             = V_y - 86.06ω  (mm/s)
```

**Right encoder (measures X-velocity):**
```
V_x_measured = V_x + ω × y_right
             = V_x + ω × (-70.0)
             = V_x - 70.0ω  (mm/s)
```

### Matrix Form

```
[V_x_enc]   [1    0    -70.00]   [V_x]
[V_y_enc] = [0    1    -86.06] × [V_y]  (mm/s)
                                  [ω]
```

Where `ω` is in rad/s, velocities in mm/s.

Converting to meters:
```
[V_x_enc]   [1    0    -0.07000]   [V_x]
[V_y_enc] = [0    1    -0.08606] × [V_y]  (m/s)
                                    [ω]
```

## Sensor Fusion Algorithm

### Three-Step Process

**Step 1: Decouple Rotation from Encoder Readings**

Remove rotation-induced velocity using gyro measurement:

```c
// Raw encoder measurements (chassis frame)
float V_x_enc = encoder_x_velocity;  // Right wheel
float V_y_enc = encoder_y_velocity;  // Left wheel

// Gyro measurement (after IAKF + compensation)
float omega_z = /* processed gyro */;

// Decouple to get true chassis velocity
float V_x_chassis = V_x_enc - (-0.07000) * omega_z;
float V_y_chassis = V_y_enc - (-0.08606) * omega_z;

// Simplified:
float V_x_chassis = V_x_enc + 0.07000 * omega_z;
float V_y_chassis = V_y_enc + 0.08606 * omega_z;
```

**Step 2: Transform to Global Frame**

Rotate chassis velocities by heading angle:

```c
float heading = quaternion_to_heading(&quaternion);

float V_x_global = V_x_chassis * cos(heading) - V_y_chassis * sin(heading);
float V_y_global = V_x_chassis * sin(heading) + V_y_chassis * cos(heading);
```

**Step 3: Integrate Position**

```c
x += V_x_global * dt;
y += V_y_global * dt;
```

## Implementation Changes

### 1. Encoder Driver (encoder_driver.h/c)

**New interface:**
```c
typedef enum {
    ENCODER_X = 0,  // Right wheel, vertical, measures X
    ENCODER_Y = 1   // Left wheel, horizontal, measures Y
} encoder_id_t;

esp_err_t encoder_driver_init(encoder_id_t encoder_id, int pin_a, int pin_b);
int32_t encoder_driver_get_count(encoder_id_t encoder_id);
void encoder_driver_reset(encoder_id_t encoder_id);
```

**Implementation:**
- Two separate `pcnt_unit_handle_t` instances
- Array-based storage: `pcnt_unit[2]`

### 2. Encoder Task (encoder_task.h/c)

**Updated state structure:**
```c
typedef struct {
    float velocity_x;      // From X-encoder (right wheel)
    float velocity_y;      // From Y-encoder (left wheel)
    int32_t pulse_count_x;
    int32_t pulse_count_y;
    uint64_t timestamp_us;
    bool valid;
    SemaphoreHandle_t mutex;
} encoder_state_t;
```

**Task loop updates:**
- Poll both encoders
- Calculate velocity for each independently
- Use separate wheel diameter parameters if needed

### 3. Configuration (config.h)

**New parameters:**
```c
// Encoder X (right wheel)
#define ENCODER_X_PIN_A  /* GPIO */
#define ENCODER_X_PIN_B  /* GPIO */
#define ENCODER_X_PPR    2048  // Pulses per revolution

// Encoder Y (left wheel)
#define ENCODER_Y_PIN_A  /* GPIO */
#define ENCODER_Y_PIN_B  /* GPIO */
#define ENCODER_Y_PPR    2048

// Geometry (in meters)
#define ENCODER_X_ROTATION_COUPLING  (0.07000f)   // |y_offset|
#define ENCODER_Y_ROTATION_COUPLING  (0.08606f)   // |x_offset|

// Wheel diameters (may differ, calibrate individually)
#define WHEEL_X_DIAMETER  (0.100f)  // meters
#define WHEEL_Y_DIAMETER  (0.100f)  // meters
```

### 4. Position Task (position_task.c)

**Replace lines 66-72 (encoder read):**
```c
// Read encoder state
float velocity_x = 0.0f, velocity_y = 0.0f;
if (xSemaphoreTake(g_context.encoder_state->mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    velocity_x = g_context.encoder_state->velocity_x;
    velocity_y = g_context.encoder_state->velocity_y;
    encoder_timestamp = g_context.encoder_state->timestamp_us;
    encoder_valid = g_context.encoder_state->valid;
    xSemaphoreGive(g_context.encoder_state->mutex);
}
```

**Replace lines 117-123 (coordinate calculation):**
```c
// 5. Decouple rotation from encoder measurements (chassis frame)
float V_x_chassis = velocity_x + ENCODER_X_ROTATION_COUPLING * omega_z_final;
float V_y_chassis = velocity_y + ENCODER_Y_ROTATION_COUPLING * omega_z_final;

// 6. Transform chassis velocities to global frame
float cos_heading = cosf(heading);
float sin_heading = sinf(heading);
float V_x_global = V_x_chassis * cos_heading - V_y_chassis * sin_heading;
float V_y_global = V_x_chassis * sin_heading + V_y_chassis * cos_heading;

// 7. Integrate position
g_context.x += V_x_global * dt;
g_context.y += V_y_global * dt;
```

**Update fail-safe logic (around line 88):**
```c
if (encoder_timeout || !encoder_valid) {
    fail_safe = true;
    confidence = 0.3f;
    velocity_x = 0.0f;
    velocity_y = 0.0f;
}
```

### 5. Modules to Remove/Simplify

**coordinate.c/.h:**
- The `coordinate_calculate_increment` function is obsolete
- X/Y encoders naturally handle omnidirectional motion
- Can be deleted or kept for reference

**fusion.c/.h:**
- Simple heading fusion may still be useful for stationary detection
- But wheel odometry heading is less meaningful with separate X/Y
- Simplify or remove

**Calibration:**
- Remove `wheel_angle` calibration (not needed)
- Add individual wheel diameter calibration
- Add coupling coefficient verification procedure

## Calibration Procedure

### 1. Wheel Diameter Calibration

**For each encoder:**
1. Move chassis in pure X or pure Y direction (known distance)
2. Record pulse counts
3. Calculate: `diameter = distance × PPR / (π × pulse_count)`

### 2. Coupling Coefficient Verification

**Pure rotation test:**
1. Rotate chassis in place (no translation)
2. Record encoder velocities and gyro angular velocity
3. Fit: `V_enc = k × ω`
4. Compare measured `k` with theoretical values:
   - X-encoder: theoretical = 70.00 mm
   - Y-encoder: theoretical = 86.06 mm
5. If difference > 5%, update config values

### 3. Orthogonality Test

**X-axis motion:**
1. Move purely in X-direction
2. Verify Y-encoder reads near zero
3. If not, check mechanical alignment

**Y-axis motion:**
1. Move purely in Y-direction
2. Verify X-encoder reads near zero
3. If not, check mechanical alignment

### 4. Combined Motion Test

**Diagonal movement:**
1. Move at 45° angle
2. Both encoders should read equal magnitudes
3. Verify position integration is correct

## Error Sources and Mitigation

### 1. Wheel Diameter Mismatch
- **Effect:** Different velocity scale factors
- **Mitigation:** Individual calibration per encoder

### 2. Encoder Alignment Error
- **Effect:** Cross-axis coupling
- **Mitigation:** Careful mechanical mounting, orthogonality tests

### 3. Coupling Coefficient Uncertainty
- **Effect:** Residual rotation-induced error
- **Mitigation:** Empirical verification, update config if needed

### 4. Wheel Slip
- **Effect:** Encoder velocity ≠ true velocity
- **Mitigation:**
  - Trust gyro for rotation
  - Detect slip via IMU/encoder consistency check
  - Lower confidence during high acceleration

### 5. Chassis Flexing
- **Effect:** Dynamic change in wheel positions
- **Mitigation:**
  - Rigid chassis design
  - Velocity-dependent coupling correction (advanced)

## Benefits of Separate X/Y Encoders

1. **Direct velocity measurement** in both axes
2. **Handles omnidirectional motion** naturally
3. **No wheel angle correction** needed
4. **Better observability** for sensor fusion
5. **Simpler coordinate transformation** (no auxiliary frame)
6. **More robust** to motion model uncertainties

## Algorithm Complexity

**Computational cost per cycle (200Hz):**
- Read 2 encoders: ~0.3ms (vs 0.2ms for 1)
- Decouple rotation: 2 multiplications, 2 additions
- Transform to global: 4 multiplications, 2 additions, 2 trig calls (can cache)
- Integration: 2 additions

**Total additional cost: < 0.2ms** (well within 5ms budget)

## Testing Strategy

### Unit Tests
1. Verify encoder driver initialization for both units
2. Test velocity calculation for each encoder
3. Verify coupling coefficient application

### Integration Tests
1. Pure X translation → Y-encoder ~0
2. Pure Y translation → X-encoder ~0
3. Pure rotation → verify decoupling works
4. Diagonal motion → both encoders active
5. Complex trajectories → position accuracy

### Hardware-in-Loop Tests
1. Known distance tests (use tape measure)
2. Closed-loop paths (should return to start)
3. Figure-8 patterns (tests rotation + translation)
4. Compare with external position reference (if available)

## Migration Path

### Phase 1: Hardware
1. Install second encoder and omniwheel
2. Wire to ESP32 GPIO pins
3. Verify mechanical alignment

### Phase 2: Driver Layer
1. Update encoder_driver for dual encoders
2. Update encoder_task for dual velocity
3. Test independently before integration

### Phase 3: Algorithm Layer
1. Update config.h parameters
2. Modify position_task processing
3. Remove obsolete coordinate/fusion modules

### Phase 4: Calibration
1. Run wheel diameter calibration
2. Verify coupling coefficients
3. Run orthogonality tests

### Phase 5: Validation
1. Compare against old system (if possible)
2. Run hardware-in-loop test suite
3. Measure position accuracy improvement

## Expected Performance

**Position accuracy:** < 1cm/m (improved from 2cm/m)
- Better X/Y velocity measurement
- More robust to motion direction changes

**Heading accuracy:** < 0.1° (unchanged)
- Still primarily gyro-based

**Update rate:** 200Hz (unchanged)
- Computational overhead is minimal

## References

- Original design: `docs/plans/2026-02-02-ops9-esp32-design.md`
- Rigid body kinematics: Standard robotics textbooks
- Encoder specifications: AMT103 datasheet

---

**Next Steps:**
1. Review and approve design
2. Create implementation plan
3. Begin Phase 1 (hardware already done)
4. Implement Phase 2-3 (software)
