# OPS9 Positioning System

ESP32-based planar positioning system using IMU (HWT901B) and encoder (AMT103) sensor fusion.

## Hardware
- ESP32 DevKit
- HWT901B IMU (UART)
- AMT103 Encoder

## Features
- 200Hz update rate
- <2cm/m position accuracy
- Multi-stage gyro error compensation
- Fail-safe mode
- UART + WiFi output

## Setup
See docs/plans/ for architecture and implementation details.
