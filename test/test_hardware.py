#!/usr/bin/env python3
"""
OPS9 Hardware-in-the-Loop Test

This script reads the binary UART output from the ESP32 and compares it
against the simulator's ground truth to validate the hardware implementation.
"""

import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
import time
import argparse

@dataclass
class UARTPacket:
    """ESP32 UART output packet structure"""
    timestamp_us: int
    x: float
    y: float
    heading: float
    confidence: float
    flags: int

    @classmethod
    def from_bytes(cls, data):
        """Parse binary packet from ESP32"""
        # Packet format: 0xAA 0x55 [length] [payload] [checksum]
        if len(data) < 23:
            return None

        if data[0] != 0xAA or data[1] != 0x55:
            return None

        # Verify checksum
        checksum = 0
        for b in data[:-1]:
            checksum ^= b

        if checksum != data[-1]:
            print(f"Checksum error: expected {checksum}, got {data[-1]}")
            return None

        # Unpack payload
        timestamp_us, x, y, heading, confidence, flags = struct.unpack('<Iffffi', data[3:-1])

        return cls(timestamp_us, x, y, heading, confidence, flags)


def read_uart_stream(port, baudrate=115200, duration=10.0):
    """Read position data from ESP32 UART"""
    print(f"Opening serial port {port} at {baudrate} baud...")

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print("Connected! Reading data...")

        packets = []
        start_time = time.time()
        buffer = bytearray()

        while time.time() - start_time < duration:
            # Read available data
            if ser.in_waiting > 0:
                buffer.extend(ser.read(ser.in_waiting))

            # Look for packet header
            while len(buffer) >= 23:
                # Find sync bytes
                if buffer[0] == 0xAA and buffer[1] == 0x55:
                    # Try to parse packet
                    packet_data = bytes(buffer[:23])
                    packet = UARTPacket.from_bytes(packet_data)

                    if packet:
                        packets.append(packet)
                        print(f"t={packet.timestamp_us/1e6:.2f}s: "
                              f"pos=({packet.x:.3f}, {packet.y:.3f}), "
                              f"heading={packet.heading*180/np.pi:.1f}°, "
                              f"conf={packet.confidence:.2f}")

                    buffer = buffer[23:]
                else:
                    # Skip byte and look for next header
                    buffer = buffer[1:]

            time.sleep(0.001)

        ser.close()
        print(f"\nReceived {len(packets)} packets")
        return packets

    except serial.SerialException as e:
        print(f"Error: {e}")
        return []


def test_with_simulated_data():
    """Test by sending simulated sensor data to ESP32 (if connected via debug interface)"""
    print("=== Simulated Sensor Data Test ===\n")
    print("This test would require:")
    print("1. ESP32 connected via JTAG/debug interface")
    print("2. Ability to inject sensor data into shared state")
    print("3. Or: Mock sensor drivers that read from file/socket")
    print("\nFor now, use the standalone Python simulator or hardware testing.")


def compare_with_ground_truth(packets, ground_truth_file=None):
    """Compare ESP32 output with ground truth from simulator"""
    if not packets:
        print("No packets to analyze")
        return

    # Extract data
    times = [p.timestamp_us / 1e6 for p in packets]
    x_vals = [p.x for p in packets]
    y_vals = [p.y for p in packets]
    headings = [p.heading * 180 / np.pi for p in packets]
    confidence = [p.confidence for p in packets]

    # Plot results
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Trajectory
    ax = axes[0, 0]
    ax.plot(x_vals, y_vals, 'b-', linewidth=2)
    ax.plot(x_vals[0], y_vals[0], 'go', markersize=10, label='Start')
    ax.plot(x_vals[-1], y_vals[-1], 'ro', markersize=10, label='End')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('ESP32 Trajectory')
    ax.legend()
    ax.grid(True)
    ax.axis('equal')

    # Position over time
    ax = axes[0, 1]
    ax.plot(times, x_vals, 'b-', label='X', linewidth=2)
    ax.plot(times, y_vals, 'r-', label='Y', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (m)')
    ax.set_title('Position Over Time')
    ax.legend()
    ax.grid(True)

    # Heading
    ax = axes[1, 0]
    ax.plot(times, headings, 'b-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Heading (degrees)')
    ax.set_title('Heading Over Time')
    ax.grid(True)

    # Confidence
    ax = axes[1, 1]
    ax.plot(times, confidence, 'g-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Confidence')
    ax.set_title('Confidence Metric')
    ax.grid(True)
    ax.set_ylim([0, 1.1])

    plt.tight_layout()
    plt.savefig('esp32_test_results.png', dpi=150)
    print("\nPlot saved to: esp32_test_results.png")
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Test OPS9 ESP32 implementation')
    parser.add_argument('--port', type=str, help='Serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--duration', type=float, default=10.0, help='Test duration (seconds)')
    parser.add_argument('--simulate', action='store_true', help='Run standalone simulator')

    args = parser.parse_args()

    if args.simulate:
        print("Running standalone simulator...")
        import subprocess
        subprocess.run(['python3', 'test/simulator.py'])
    elif args.port:
        print(f"=== OPS9 Hardware Test ===\n")
        packets = read_uart_stream(args.port, args.baud, args.duration)
        compare_with_ground_truth(packets)
    else:
        print("=== OPS9 Testing Options ===\n")
        print("1. Test with hardware:")
        print("   python3 test/test_hardware.py --port /dev/ttyUSB0 --duration 20")
        print()
        print("2. Run standalone simulator:")
        print("   python3 test/test_hardware.py --simulate")
        print("   OR")
        print("   python3 test/simulator.py")
        print()
        print("3. Flash and monitor ESP32:")
        print("   source .venv/bin/activate")
        print("   pio run --target upload")
        print("   pio device monitor")
        print()
        print("Note: Hardware testing requires ESP32 connected via USB")


if __name__ == '__main__':
    main()
