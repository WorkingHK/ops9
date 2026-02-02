#!/usr/bin/env python3
"""
OPS9 Positioning System Simulator

Simulates IMU and encoder sensors to test the positioning algorithm
without requiring physical hardware.
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
import struct
import time

@dataclass
class SimulatedRobot:
    """Simulates a robot moving in 2D space"""
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0  # radians
    velocity: float = 0.0  # m/s
    angular_velocity: float = 0.0  # rad/s

    # Sensor noise parameters
    gyro_noise_std: float = 0.001  # rad/s
    gyro_bias: np.ndarray = None
    accel_noise_std: float = 0.1  # m/s²
    encoder_noise_std: float = 0.01  # m/s
    temperature: float = 25.0  # °C

    def __post_init__(self):
        if self.gyro_bias is None:
            self.gyro_bias = np.array([0.001, -0.002, 0.003])  # rad/s

    def update(self, dt: float, linear_accel: float = 0.0, angular_accel: float = 0.0):
        """Update robot state with physics"""
        # Update velocities
        self.velocity += linear_accel * dt
        self.angular_velocity += angular_accel * dt

        # Update position and heading
        self.heading += self.angular_velocity * dt
        self.x += self.velocity * np.cos(self.heading) * dt
        self.y += self.velocity * np.sin(self.heading) * dt

        # Normalize heading to [-pi, pi]
        self.heading = np.arctan2(np.sin(self.heading), np.cos(self.heading))

    def get_imu_reading(self):
        """Get simulated IMU reading with noise and bias"""
        omega_x = 0.0 + np.random.normal(0, self.gyro_noise_std) + self.gyro_bias[0]
        omega_y = 0.0 + np.random.normal(0, self.gyro_noise_std) + self.gyro_bias[1]
        omega_z = self.angular_velocity + np.random.normal(0, self.gyro_noise_std) + self.gyro_bias[2]

        # Simulate acceleration (simplified - just gravity and motion)
        accel_x = np.random.normal(0, self.accel_noise_std)
        accel_y = np.random.normal(0, self.accel_noise_std)
        accel_z = 9.8 + np.random.normal(0, self.accel_noise_std)

        return {
            'omega_x': omega_x,
            'omega_y': omega_y,
            'omega_z': omega_z,
            'accel_x': accel_x,
            'accel_y': accel_y,
            'accel_z': accel_z,
            'temperature': self.temperature
        }

    def get_encoder_reading(self):
        """Get simulated encoder reading with noise"""
        velocity = self.velocity + np.random.normal(0, self.encoder_noise_std)
        return {'velocity': velocity}


class PositioningAlgorithm:
    """Implements the OPS9 positioning algorithm"""

    def __init__(self):
        # IAKF state
        self.iakf_x = 0.0
        self.iakf_P = 1.0
        self.iakf_Q = 0.001
        self.iakf_R = 0.01

        # Quaternion state
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # [q0, q1, q2, q3]

        # Position state
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

        # Calibration
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.wheel_angle = 0.0

    def iakf_update(self, measurement):
        """Improved Adaptive Kalman Filter"""
        # Prediction
        P_pred = self.iakf_P + self.iakf_Q

        # Innovation
        innovation = measurement - self.iakf_x

        # Adaptive R
        R_adaptive = self.iakf_R * (1.0 + abs(innovation))

        # Kalman gain
        K = P_pred / (P_pred + R_adaptive)

        # Update
        self.iakf_x = self.iakf_x + K * innovation
        self.iakf_P = (1.0 - K) * P_pred

        return self.iakf_x

    def quaternion_normalize(self):
        """Normalize quaternion"""
        norm = np.linalg.norm(self.q)
        if norm > 0:
            self.q /= norm

    def quaternion_update_rk2(self, omega_x, omega_y, omega_z, dt):
        """2nd-order Runge-Kutta quaternion integration"""
        # k1 = f(q, omega)
        k1 = 0.5 * np.array([
            -self.q[1]*omega_x - self.q[2]*omega_y - self.q[3]*omega_z,
             self.q[0]*omega_x + self.q[2]*omega_z - self.q[3]*omega_y,
             self.q[0]*omega_y - self.q[1]*omega_z + self.q[3]*omega_x,
             self.q[0]*omega_z + self.q[1]*omega_y - self.q[2]*omega_x
        ])

        # q_mid = q + k1 * dt/2
        q_mid = self.q + k1 * dt * 0.5

        # k2 = f(q_mid, omega)
        k2 = 0.5 * np.array([
            -q_mid[1]*omega_x - q_mid[2]*omega_y - q_mid[3]*omega_z,
             q_mid[0]*omega_x + q_mid[2]*omega_z - q_mid[3]*omega_y,
             q_mid[0]*omega_y - q_mid[1]*omega_z + q_mid[3]*omega_x,
             q_mid[0]*omega_z + q_mid[1]*omega_y - q_mid[2]*omega_x
        ])

        # q = q + k2 * dt
        self.q += k2 * dt
        self.quaternion_normalize()

    def quaternion_to_heading(self):
        """Extract heading from quaternion"""
        return np.arctan2(2.0 * (self.q[0]*self.q[3] + self.q[1]*self.q[2]),
                         1.0 - 2.0 * (self.q[2]**2 + self.q[3]**2))

    def update(self, imu_data, encoder_data, dt):
        """Main algorithm update"""
        # Apply gyro bias calibration
        omega_x = imu_data['omega_x'] - self.gyro_bias[0]
        omega_y = imu_data['omega_y'] - self.gyro_bias[1]
        omega_z = imu_data['omega_z'] - self.gyro_bias[2]

        # IAKF filtering
        omega_z_filtered = self.iakf_update(omega_z)

        # Threshold processing (dead zone)
        if abs(omega_z_filtered) < 0.01:
            omega_z_filtered = 0.0

        # Attitude solving
        self.quaternion_update_rk2(omega_x, omega_y, omega_z_filtered, dt)
        self.heading = self.quaternion_to_heading()

        # Coordinate calculation
        velocity = encoder_data['velocity']
        corrected_heading = self.heading + self.wheel_angle

        dx = velocity * np.cos(corrected_heading) * dt
        dy = velocity * np.sin(corrected_heading) * dt

        self.x += dx
        self.y += dy

        return {
            'x': self.x,
            'y': self.y,
            'heading': self.heading
        }


def run_simulation():
    """Run the simulation"""
    print("=== OPS9 Positioning System Simulator ===\n")

    # Simulation parameters
    dt = 0.005  # 5ms (200Hz)
    duration = 20.0  # seconds
    steps = int(duration / dt)

    # Create robot and algorithm
    robot = SimulatedRobot()
    algo = PositioningAlgorithm()

    # Storage for plotting
    time_data = []
    true_x, true_y, true_heading = [], [], []
    est_x, est_y, est_heading = [], [], []

    print("Running simulation...")
    print(f"Duration: {duration}s, Update rate: {1/dt:.0f}Hz, Steps: {steps}\n")

    # Define motion profile
    # 0-5s: Move forward at 0.5 m/s
    # 5-10s: Turn left (90 degrees) while moving
    # 10-15s: Move forward at 0.5 m/s
    # 15-20s: Stop and rotate in place

    for step in range(steps):
        t = step * dt

        # Motion commands
        if t < 5.0:
            linear_accel = 0.5 if robot.velocity < 0.5 else 0.0
            angular_accel = 0.0
        elif t < 10.0:
            linear_accel = 0.0
            angular_accel = 0.3 if robot.angular_velocity < 0.3 else 0.0
        elif t < 15.0:
            linear_accel = 0.0
            angular_accel = -0.3 if robot.angular_velocity > 0.0 else 0.0
        else:
            linear_accel = -0.5 if robot.velocity > 0.0 else 0.0
            angular_accel = 0.5 if robot.angular_velocity < 0.5 else 0.0

        # Update robot physics
        robot.update(dt, linear_accel, angular_accel)

        # Get sensor readings
        imu_data = robot.get_imu_reading()
        encoder_data = robot.get_encoder_reading()

        # Update algorithm
        est_state = algo.update(imu_data, encoder_data, dt)

        # Store data for plotting (every 10th sample to reduce data)
        if step % 10 == 0:
            time_data.append(t)
            true_x.append(robot.x)
            true_y.append(robot.y)
            true_heading.append(robot.heading)
            est_x.append(est_state['x'])
            est_y.append(est_state['y'])
            est_heading.append(est_state['heading'])

        # Print progress
        if step % 1000 == 0:
            pos_err = ((robot.x-est_state['x'])**2 + (robot.y-est_state['y'])**2)**0.5
            print(f"t={t:.1f}s: True pos=({robot.x:.3f}, {robot.y:.3f}), "
                  f"Est pos=({est_state['x']:.3f}, {est_state['y']:.3f}), "
                  f"Error={pos_err:.3f}m")

    # Calculate final errors
    final_pos_error = np.sqrt((robot.x - algo.x)**2 + (robot.y - algo.y)**2)
    final_heading_error = abs(robot.heading - algo.heading) * 180 / np.pi

    print(f"\n=== Simulation Complete ===")
    print(f"Final position error: {final_pos_error:.4f} m")
    print(f"Final heading error: {final_heading_error:.2f}°")

    # Plot results
    plot_results(time_data, true_x, true_y, true_heading, est_x, est_y, est_heading)


def plot_results(time_data, true_x, true_y, true_heading, est_x, est_y, est_heading):
    """Plot simulation results"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Trajectory plot
    ax = axes[0, 0]
    ax.plot(true_x, true_y, 'b-', label='True', linewidth=2)
    ax.plot(est_x, est_y, 'r--', label='Estimated', linewidth=2)
    ax.plot(true_x[0], true_y[0], 'go', markersize=10, label='Start')
    ax.plot(true_x[-1], true_y[-1], 'ro', markersize=10, label='End')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('2D Trajectory')
    ax.legend()
    ax.grid(True)
    ax.axis('equal')

    # Position error over time
    ax = axes[0, 1]
    pos_error = [np.sqrt((tx-ex)**2 + (ty-ey)**2)
                 for tx, ty, ex, ey in zip(true_x, true_y, est_x, est_y)]
    ax.plot(time_data, pos_error, 'r-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position Error (m)')
    ax.set_title('Position Error Over Time')
    ax.grid(True)

    # Heading comparison
    ax = axes[1, 0]
    ax.plot(time_data, np.array(true_heading) * 180/np.pi, 'b-', label='True', linewidth=2)
    ax.plot(time_data, np.array(est_heading) * 180/np.pi, 'r--', label='Estimated', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Heading (degrees)')
    ax.set_title('Heading Over Time')
    ax.legend()
    ax.grid(True)

    # Heading error
    ax = axes[1, 1]
    heading_error = [abs(th-eh) * 180/np.pi
                     for th, eh in zip(true_heading, est_heading)]
    ax.plot(time_data, heading_error, 'r-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Heading Error (degrees)')
    ax.set_title('Heading Error Over Time')
    ax.grid(True)

    plt.tight_layout()
    plt.savefig('simulation_results.png', dpi=150)
    print("\nPlot saved to: simulation_results.png")
    plt.show()


if __name__ == '__main__':
    run_simulation()
