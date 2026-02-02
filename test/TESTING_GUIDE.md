# OPS9 Testing Guide

## Testing Options

You have three ways to test the OPS9 positioning system:

### 1. 🖥️ Standalone Simulator (No Hardware Required)

**What it does**: Runs a complete software simulation with synthetic sensor data

```bash
source .venv/bin/activate
python3 test/simulator.py
```

**Output**:
- Console: Progress updates every 5 seconds
- File: `simulation_results.png` with 4 plots
- Shows: Position error, heading error, trajectory

**Use case**: Algorithm validation, parameter tuning, development

---

### 2. 🔌 Hardware-in-the-Loop Testing (ESP32 Required)

**What it does**: Reads real UART output from ESP32 and visualizes it

```bash
source .venv/bin/activate

# Find your serial port
ls /dev/ttyUSB*  # Linux
ls /dev/tty.*    # macOS

# Run test (replace with your port)
python3 test/test_hardware.py --port /dev/ttyUSB0 --duration 20
```

**Output**:
- Console: Real-time position updates from ESP32
- File: `esp32_test_results.png` with trajectory and metrics
- Shows: Actual hardware performance

**Use case**: Hardware validation, real-world testing

---

### 3. 🚀 Direct ESP32 Testing (Hardware Only)

**What it does**: Flash firmware and monitor serial output

```bash
source .venv/bin/activate

# Flash firmware
pio run --target upload

# Monitor output
pio device monitor
```

**What you'll see**:
```
=== OPS9 Positioning System Starting ===
IMU UART initialized on UART1 (RX:16, TX:17, baud:115200)
Encoder driver initialized on GPIO25/26
Position task started on core 1
=== All tasks started, system running ===
Position: x=0.000 y=0.000 heading=0.0° conf=1.00 fail=0
```

**Use case**: System integration, calibration, deployment

---

## Quick Start Testing

### Option A: Just want to see it work? (No hardware)

```bash
source .venv/bin/activate
python3 test/simulator.py
```

This runs a 20-second simulation and shows you plots!

### Option B: Have ESP32 hardware?

```bash
source .venv/bin/activate

# 1. Flash firmware
pio run --target upload

# 2. Monitor (Ctrl+C to exit)
pio device monitor

# 3. Run calibration
# In monitor, send: CAL_GYRO
# Wait 30 seconds
# Send: CAL_STATUS
```

### Option C: Want to analyze hardware output?

```bash
source .venv/bin/activate

# Record data from ESP32
python3 test/test_hardware.py --port /dev/ttyUSB0 --duration 20

# View: esp32_test_results.png
```

---

## Simulator Features

The Python simulator (`test/simulator.py`) includes:

✅ **Realistic sensor simulation**:
- IMU with gyro noise and bias
- Encoder with velocity noise
- Temperature effects (constant for now)

✅ **Complex motion profile**:
- Forward acceleration
- Turning while moving
- Deceleration
- Rotation in place

✅ **Algorithm validation**:
- IAKF filtering
- Quaternion integration
- Sensor fusion
- Position tracking

✅ **Visualization**:
- 2D trajectory plot
- Position error over time
- Heading comparison
- Heading error analysis

---

## Test Results Interpretation

### Simulator Output

```
t=0.0s: True pos=(0.000, 0.000), Est pos=(0.000, -0.000), Error=0.000m
t=5.0s: True pos=(2.254, 0.000), Est pos=(2.252, -0.000), Error=0.002m
...
Final position error: 0.0581 m
Final heading error: 1.50°
```

**Good results**:
- Position error < 10 cm after 20s
- Heading error < 5°
- No divergence

**Bad results**:
- Position error > 50 cm
- Heading error > 10°
- Rapid error growth

### Hardware Output

```
Position: x=1.234 y=0.567 heading=45.2° conf=0.95 fail=0
```

**Fields**:
- `x, y`: Position in meters
- `heading`: Direction in degrees
- `conf`: Confidence (0.0-1.0)
- `fail`: Fail-safe mode (0=normal, 1=degraded)

**Good signs**:
- `conf` close to 1.0
- `fail=0`
- Smooth position changes

**Warning signs**:
- `conf` < 0.5
- `fail=1`
- Erratic position jumps

---

## Troubleshooting

### Simulator Issues

**Problem**: `ModuleNotFoundError: No module named 'numpy'`
```bash
source .venv/bin/activate
pip install numpy matplotlib
```

**Problem**: No plot appears
- Check if `simulation_results.png` was created
- Try: `python3 test/simulator.py` (without display)

### Hardware Issues

**Problem**: `Serial port not found`
```bash
# Linux: Check permissions
sudo usermod -a -G dialout $USER
# Logout and login again

# Find port
ls /dev/ttyUSB*
```

**Problem**: `No data received`
- Check USB connection
- Verify baud rate (115200)
- Try: `pio device monitor` first

**Problem**: `Checksum errors`
- Baud rate mismatch
- Cable issues
- EMI interference

---

## Advanced Testing

### Custom Motion Profile

Edit `test/simulator.py` and modify the motion commands:

```python
# Around line 230
if t < 5.0:
    linear_accel = 1.0  # Faster acceleration
    angular_accel = 0.0
```

### Longer Duration

```bash
# Edit simulator.py, line 210
duration = 60.0  # 60 seconds instead of 20
```

### Add Sensor Noise

```python
# Edit simulator.py, line 25
gyro_noise_std: float = 0.005  # More noise
```

---

## Next Steps

After testing:

1. ✅ **Simulator passes** → Flash to hardware
2. ✅ **Hardware works** → Run calibration
3. ✅ **Calibration done** → Test with motion
4. ✅ **Motion tracking good** → Deploy to robot!

---

## Files Generated

- `simulation_results.png` - Simulator output plots
- `esp32_test_results.png` - Hardware test plots
- `test/SIMULATOR_REPORT.md` - Detailed test report

---

**Ready to test? Start with the simulator!**

```bash
source .venv/bin/activate
python3 test/simulator.py
```
